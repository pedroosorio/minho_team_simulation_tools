#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "multicastpp.h"


///
/// SIGNAL newRobotInformationReceived is triggered when new information is received from RTDB
///

MainWindow::MainWindow(bool isOfficialField, Multicastpp *coms, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    for(int i=0;i<NROBOTS;i++) robwidgets[i] = NULL;
    ui->setupUi(this);
    robwidgetsReady = false;
    refboxConnected = false;
    refboxSocket = NULL;
    ui->statusBar->showMessage("Loading resources ...");
    setupGraphicsUI();
    connectToRefBox();

    isCyan = false;
    on_bt_team_clicked();

    mBsInfo.roles.resize(NROBOTS);
    mBsInfo.gamestate = sSTOPPED;
    mBsInfo.posxside = false;
    on_bt_side_clicked();

    rtdb = coms;
    this->isOfficialField = isOfficialField;
    initGazeboBaseStationWorld();
    connect(ui->gzwidget,SIGNAL(newFrameRendered()),this,SLOT(setup3DVisualPtrs()));
    ui->gzwidget->init("mtbasestation");
    ui->gzwidget->setGrid(false);
    ui->gzwidget->setAllControlsMode(false);

    bsBallVisual = NULL;
    mBsInfo.agent_id = 6;
    run_gz = NULL;
    for(int i=0;i<NROBOTS;i++) {
        robotState[i] = false; robotReceivedPackets[i]=0;
        robotVisuals[i] = ballVisuals[i] = NULL;
        recvFreqs[i] = 0;
    }

    robotStateDetector = new QTimer();
    sendDataTimer = new QTimer();
    connect(robotStateDetector,SIGNAL(timeout()),this,SLOT(detectRobotsState()));
    connect(sendDataTimer,SIGNAL(timeout()),this,SLOT(sendBaseStationUpdate()));
    robotStateDetector->start(300);
    on_comboBox_activated(0);

    // Signal test
    connect(ui->gzwidget,SIGNAL(modelClicked(QString)),this,SLOT(printSlot(QString)));
    connect(ui->gzwidget,SIGNAL(modelReleased(QString)),this,SLOT(printSlot(QString)));
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::setupGraphicsUI()
{
    QVBoxLayout *layouts = new QVBoxLayout[5];
    ui->frame->setLayout(&layouts[0]);
    layouts[0].addWidget(ui->r1widget);
    layouts[0].setSizeConstraint(QLayout::SetMaximumSize);
    layouts[0].setContentsMargins(0,0,0,0);
    ui->r1widget->setRobotId(1);
    ui->r1widget->setWidgetState(false);
    robwidgets[0] = ui->r1widget;
    //--
    ui->frame_2->setLayout(&layouts[1]);
    layouts[1].addWidget(ui->r2widget);
    layouts[1].setSizeConstraint(QLayout::SetMaximumSize);
    layouts[1].setContentsMargins(0,0,0,0);
    ui->r2widget->setRobotId(2);
    ui->r2widget->setWidgetState(false);
    robwidgets[1] = ui->r2widget;
    //--
    ui->frame_3->setLayout(&layouts[2]);
    layouts[2].addWidget(ui->r3widget);
    layouts[2].setSizeConstraint(QLayout::SetMaximumSize);
    layouts[2].setContentsMargins(0,0,0,0);
    ui->r3widget->setRobotId(3);
    ui->r3widget->setWidgetState(false);
    robwidgets[2] = ui->r3widget;
    //--
    ui->frame_4->setLayout(&layouts[3]);
    layouts[3].addWidget(ui->r4widget);
    layouts[3].setSizeConstraint(QLayout::SetMaximumSize);
    layouts[3].setContentsMargins(0,0,0,0);
    ui->r4widget->setRobotId(4);
    ui->r4widget->setWidgetState(false);
    robwidgets[3] = ui->r4widget;
    //--
    ui->frame_5->setLayout(&layouts[4]);
    layouts[4].addWidget(ui->r5widget);
    layouts[4].setSizeConstraint(QLayout::SetMaximumSize);
    layouts[4].setContentsMargins(0,0,0,0);
    ui->r5widget->setRobotId(5);
    ui->r5widget->setWidgetState(false);
    robwidgets[4] = ui->r5widget;

    robwidgetsReady = true;
    ui->lb_refstate->setText("RefBox ●");
    ui->lb_refstate->setStyleSheet("color:red;");
    ui->lb_refstate->setFont(ui->bt_team->font());
}

void MainWindow::updateAgentInfo(void *packet)
{
    // deserialize message
    interAgentInfo agent_data;
    udp_packet *data = (udp_packet*)packet;
    if(!isAgentInfoMessage(data)) return;
    deserializeROSMessage<interAgentInfo>(data,&agent_data);
    delete(data);

    if(agent_data.agent_id<1 || agent_data.agent_id>(NROBOTS));
    else {
      recvFreqs[agent_data.agent_id-1] = 1000.0/(float)data_timers[agent_data.agent_id-1].elapsed();
      data_timers[agent_data.agent_id-1].start();
      robots[agent_data.agent_id-1] = agent_data;
      robotReceivedPackets[agent_data.agent_id-1]++;
      emit newRobotInformationReceived(agent_data.agent_id);
    }
    return;
}

bool MainWindow::isAgentInfoMessage(udp_packet *packet)
{
    UInt8 msg;
    ros::serialization::IStream istream(packet->packet, sizeof(msg.data));
    ros::serialization::deserialize(istream, msg);
    if(msg.data==1) return true;
    else return false;
}

void MainWindow::sendBaseStationUpdate()
{
    // Compute stuff

    // Update Graphics
    updateGraphics();
    ui->statusBar->showMessage("Ξ Rendering at "+QString::number(ui->gzwidget->getAverageFPS())+" fps");
    // Update Roles from robot widgets
    for(unsigned int rob=0;rob<NROBOTS;rob++) {
        mBsInfo.roles[rob] = robwidgets[rob]->getCurrentRole();
        if(robotState[rob] && robwidgets[rob]) robwidgets[rob]->updateInformation(robots[rob].hardware_info,recvFreqs[rob]);
    }
    // Send information to Robots
    sendInfoOverMulticast();
}

void MainWindow::sendInfoOverMulticast()
{
    uint8_t *packet;
    uint32_t packet_size, sentbytes;
    serializeROSMessage<baseStationInfo>(&mBsInfo,&packet,&packet_size);
    // Send packet of size packet_size through UDP
    sentbytes = rtdb->sendData(packet,packet_size);
    if (sentbytes != packet_size){
        ROS_ERROR("Failed to send a packet.");
    }
}

void MainWindow::initGazeboBaseStationWorld()
{
    std::string gazebouri = "http://127.0.0.1:11346";
    setenv("GAZEBO_MASTER_URI",gazebouri.c_str(),1);
    runGzServer();
}

bool MainWindow::runGzServer()
{
    boost::process::context ctx;
    std::string worldfilename = "";
    if(isOfficialField) worldfilename = "bs_official.world";
    else worldfilename = "bs_lar.world";
    // start gzserver
    std::vector<std::string> args;
    std::string gz = "/usr/bin/gzserver";
    args.push_back(worldfilename);
    // add mathching world file
    if(run_gz!=NULL){run_gz->terminate(true); delete run_gz; run_gz = NULL;}
    std::map<int,boost::process::handle> a;
    run_gz = new boost::process::child(0,a);
    (*run_gz) = boost::process::create_child(gz, args, ctx);
    usleep(5000000);
}

void MainWindow::setVisibilityRobotGraphics(int robot_id, bool isVisible)
{
    if(robot_id<0||robot_id>NROBOTS-1) return;
    float transp = 1.0;
    if(isVisible) transp = 0.0;
    if(robotVisuals[robot_id]){
        robotVisuals[robot_id]->SetTransparency(transp);
        if(ballVisuals[robot_id]) ballVisuals[robot_id]->SetTransparency(transp);
    } else return;
}

void MainWindow::setVisibilityBsBall(bool isVisible)
{
    float transp = 1.0;
    if(isVisible) transp = 0.0;
    if(bsBallVisual) bsBallVisual->SetTransparency(transp);
}

void MainWindow::updateGraphics()
{
    int onRobots = 0;
    for(int i=0;i<NROBOTS;i++){
        robwidgets[i]->setWidgetState(robotState[i]);
        if(robotState[i]){
            // show stuff from robot i
            setVisibilityRobotGraphics(i,true);
                // update robot position
                setRobotPose(i+1,robots[i].agent_info.robot_info.robot_pose.x,
                             robots[i].agent_info.robot_info.robot_pose.y,
                             robots[i].agent_info.robot_info.robot_pose.z);
                // if sees ball, update ball position, else, hide ball
                if(robots[i].agent_info.robot_info.sees_ball){
                    float ballHeight = 0.11;
                    if(robots[i].is_goalkeeper) ballHeight = robots[i].agent_info.robot_info.ball_position.z;
                        ballHeight = robots[i].agent_info.robot_info.ball_position.z;
                    setBallPosition(i+1,robots[i].agent_info.robot_info.ball_position.x,
                                        robots[i].agent_info.robot_info.ball_position.y,
                                        ballHeight);
                } else if(ballVisuals[i]) ballVisuals[i]->SetTransparency(1.0);

            onRobots++;
        } else {
            // hide stuff from robot i
            setVisibilityRobotGraphics(i,false);
            mBsInfo.roles[i] = rSTOP;
            data_timers[i].restart();
        }
    }

    if(onRobots>0){
        setVisibilityBsBall(true);
    } else setVisibilityBsBall(false);
}

void MainWindow::setCameraPose(float x, float y, float z, float yaw, float pitch, float roll)
{
    ui->gzwidget->setCameraPose(Vector3d(x,y,z),Vector3d(yaw,pitch,roll));
}

void MainWindow::setRobotPose(int robot_id, float x, float y, float z)
{
    if(robot_id<1||robot_id>NROBOTS) return;
    std::string robotprefix = "robot_";
    std::string modelName = robotprefix+std::to_string(robot_id);
    ui->gzwidget->setModelPoseInWorld(modelName,Vector3d(x,y,z));
}

void MainWindow::setBallPosition(int ball_id, float x, float y, float z)
{
    if(ball_id<0||ball_id>NROBOTS) return;
    std::string ballprefix = "ball_";
    std::string bsballname = "ball_bs";
    std::string modelName = bsballname;
    if(ball_id>0) modelName = ballprefix+std::to_string(ball_id);
    ui->gzwidget->setModelPoseInWorld(modelName,Vector3d(x,y,z));
}

bool MainWindow::connectToRefBox()
{
    if(refboxSocket) {
        refboxSocket->close();
        refboxSocket->abort();
        delete refboxSocket;
    }

    refboxSocket = new QTcpSocket();
    QString refboxIP = "127.0.0.1";
    // red iptable.cfg from common
    std::string ipFilePath = getenv("HOME");
    std::string line = "";
    ipFilePath += "/Common/iptable.cfg";
    std::ifstream file; file.open(ipFilePath);
    if(file.is_open()){
        while (getline(file,line)){
            if(line.size()>0 && line[0]!='#'){
                if(line.find("RB")!=std::string::npos){
                    refboxIP = QString::fromStdString(line.substr(0,line.find(" ")));
                    ROS_INFO("Found Refbox IP config at iptable.cfg - %s",refboxIP.toStdString().c_str());
                }
            }
        }
        file.close();
    } else ROS_ERROR("Failed to read iptable.cfg");
    ui->lb_refstate->setText(QString("RefBox [")+refboxIP+QString("] ●"));
    ui->lb_refstate->setFont(ui->bt_team->font());
    if(refboxIP=="127.0.0.1") ROS_INFO("Setting RefBox IP as localhost.");

    int enableKeepAlive = 1;
    int fd = refboxSocket->socketDescriptor();
    setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &enableKeepAlive, sizeof(enableKeepAlive));
    int maxIdle = 5; /* seconds */
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &maxIdle, sizeof(maxIdle));
    int count = 3;  // send up to 3 keepalive packets out, then disconnect if no response
    setsockopt(fd, SOL_TCP, TCP_KEEPCNT, &count, sizeof(count));
    int interval = 2;   // send a keepalive packet out every 2 seconds (after the 5 second idle period)
    setsockopt(fd, SOL_TCP, TCP_KEEPINTVL, &interval, sizeof(interval));

    refboxSocket->connectToHost(QHostAddress(refboxIP),28097);
    connect(refboxSocket,SIGNAL(readyRead()),this,SLOT(onRefBoxData()));
    connect(refboxSocket,SIGNAL(disconnected()),this,SLOT(onRefBoxDisconnection()));
}

void MainWindow::detectRobotsState()
{
    for(int i=0;i<NROBOTS;i++) {
        if(robotReceivedPackets[i]>6) robotState[i] = true;
        else robotState[i] = false;
        robotReceivedPackets[i] = 0;
    }
}

void MainWindow::setup3DVisualPtrs()
{
    scene = ui->gzwidget->getScene();
    std::string robotprefix = "robot_";
    std::string ballprefix = "ball_";
    std::string bsballname = "ball_bs";

    bsBallVisual = scene->GetVisual(bsballname);
    if(bsBallVisual!=NULL){
        bsBallVisual = bsBallVisual->GetRootVisual();
        disconnect(ui->gzwidget,SIGNAL(newFrameRendered()),this,SLOT(setup3DVisualPtrs()));
        sendDataTimer->start(CYCLE_TIME);
    }else return;

    for(int i=0;i<NROBOTS;i++){
        robotVisuals[i] = scene->GetVisual(robotprefix+std::to_string(i+1))->GetRootVisual();
        ballVisuals[i] = scene->GetVisual(ballprefix+std::to_string(i+1))->GetRootVisual();
    }
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    ui->gzwidget->close();
    system("pkill -f \"gzserver bs_\" ");
    int status = 0;
    if(run_gz)waitpid(run_gz->get_id(), &status, WNOHANG);
    event->accept();
}

template<typename Message>
void MainWindow::deserializeROSMessage(udp_packet *packet, Message *msg)
{
    ros::serialization::IStream istream(packet->packet, packet->packet_size);
    ros::serialization::deserialize(istream, *msg);
}

template<typename Message>
void MainWindow::serializeROSMessage(Message *msg, uint8_t **packet, uint32_t *packet_size)
{
    uint32_t serial_size = ros::serialization::serializationLength( *msg );
    serialization_buffer.reset(new uint8_t[serial_size]);
    (*packet_size) = serial_size;
    ros::serialization::OStream stream( serialization_buffer.get(), serial_size );
    ros::serialization::serialize( stream, *msg);
    (*packet) = serialization_buffer.get();
}

void MainWindow::on_bt_team_clicked()
{
    isCyan = !isCyan;
    QPalette pal = ui->bt_team->palette();
    if(isCyan){ pal.setColor(QPalette::Button,QColor(Qt::cyan)); ui->bt_team->setText("Team CYAN"); }
    else { pal.setColor(QPalette::Button,QColor(Qt::magenta)); ui->bt_team->setText("Team MAGENTA"); }
    ui->bt_team->setPalette(pal);
}

void MainWindow::on_bt_side_clicked()
{
    mBsInfo.posxside = !mBsInfo.posxside;
    QPalette pal = ui->bt_side->palette();
    if(mBsInfo.posxside){ pal.setColor(QPalette::Button,QColor(Qt::yellow)); ui->bt_side->setText("Right Side"); }
    else { pal.setColor(QPalette::Button,QColor(Qt::red)); ui->bt_side->setText("Left Side"); }
    ui->bt_side->setPalette(pal);
}

void MainWindow::on_bt_conref_clicked()
{
    connectToRefBox();
}

void MainWindow::onRefBoxData()
{
    QString command = refboxSocket->readAll();

    if(command == "S"){ // stop,end part or end half
        mBsInfo.gamestate = sSTOPPED;
    } else if(command == "W"){ // on connection with refbox
        refboxConnected = true;
        ui->lb_refstate->setStyleSheet("color:green;");
        ui->lb_refstate->setFont(ui->bt_team->font());
    }else if(command == "e" || command == "h"){ // end part or end half
        mBsInfo.gamestate = sSTOPPED;
        if(command=="h") on_bt_side_clicked();
    } else if(command == "L"){ // parking
        mBsInfo.gamestate = sPARKING;
        if(command=="h") on_bt_side_clicked();
    } else if(command == "s" || command == "1s" || command == "2s"){ //start
        mBsInfo.gamestate++;
    } else if(command == "K" || command == "k"){ // kickoff
        if(isCyan && command[0].isUpper()) mBsInfo.gamestate = sPRE_OWN_KICKOFF;
        else if(!isCyan && command[0].isLower()) mBsInfo.gamestate = sPRE_OWN_KICKOFF;
        else mBsInfo.gamestate = sPRE_THEIR_KICKOFF;
    } else if(command == "F" || command == "f"){ // freekick
        if(isCyan && command[0].isUpper()) mBsInfo.gamestate = sPRE_OWN_FREEKICK;
        else if(!isCyan && command[0].isLower()) mBsInfo.gamestate = sPRE_OWN_FREEKICK;
        else mBsInfo.gamestate = sPRE_THEIR_FREEKICK;
    } else mBsInfo.gamestate = sSTOPPED;
}

void MainWindow::onRefBoxDisconnection()
{
    mBsInfo.gamestate = sSTOPPED;
    ui->lb_refstate->setStyleSheet("color:red;");
    ui->lb_refstate->setFont(ui->bt_team->font());
}

void MainWindow::on_comboBox_activated(int index)
{
    switch(index){
        case 0:{
            if(isOfficialField) setCameraPose(0.0,-22,12,0,0.55,1.58);
            break;
        }
        case 1:{
            if(isOfficialField) setCameraPose(18.5,-19,10,0,0.41,2.27);
            break;
        }
        case 2:{
            if(isOfficialField) setCameraPose(-17,-19.5,10,0,0.41,0.9);
            break;
        }
        case 3:{
            if(isOfficialField) setCameraPose(0.0,0.0,33,0,1.57,1.57);
            break;
        }
    }
}
