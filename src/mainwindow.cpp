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
    ui->setupUi(this);
    rtdb = coms;
    this->isOfficialField = isOfficialField;
    initGazeboBaseStationWorld();
    connect(ui->widget,SIGNAL(newFrameRendered()),this,SLOT(setup3DVisualPtrs()));
    ui->widget->init("mtbasestation");
    ui->widget->setGrid(false);
    ui->widget->setAllControlsMode(true);

    bsBallVisual = NULL;
    mBsInfo.agent_id = 6;
    run_gz = NULL;
    for(int i=0;i<NROBOTS;i++) {
        robotState[i] = false; robotReceivedPackets[i]=0;
        robotVisuals[i] = ballVisuals[i] = NULL;
    }

    robotStateDetector = new QTimer();
    sendDataTimer = new QTimer();
    connect(robotStateDetector,SIGNAL(timeout()),this,SLOT(detectRobotsState()));
    connect(sendDataTimer,SIGNAL(timeout()),this,SLOT(sendBaseStationUpdate()));
    robotStateDetector->start(300);
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
      robots[agent_data.agent_id-1] = agent_data;
      robotReceivedPackets[agent_data.agent_id-1]++;
      emit newRobotInformationReceived();
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
    // Send information to Robots
    //sendInfoOverMulticast();
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
        }
    }

    if(onRobots>0){
        setVisibilityBsBall(true);
    } else setVisibilityBsBall(false);
}

void MainWindow::setCameraPose(float x, float y, float z, float yaw, float pitch, float roll)
{
    ui->widget->setCameraPose(Vector3d(x,y,z),Vector3d(yaw,pitch,roll));
}

void MainWindow::setRobotPose(int robot_id, float x, float y, float z)
{
    if(robot_id<1||robot_id>NROBOTS) return;
    std::string robotprefix = "robot_";
    std::string modelName = robotprefix+std::to_string(robot_id);
    ui->widget->setModelPoseInWorld(modelName,Vector3d(x,y,z));
}

void MainWindow::setBallPosition(int ball_id, float x, float y, float z)
{
    if(ball_id<0||ball_id>NROBOTS) return;
    std::string ballprefix = "ball_";
    std::string bsballname = "ball_bs";
    std::string modelName = bsballname;
    if(ball_id>0) modelName = ballprefix+std::to_string(ball_id);
    ui->widget->setModelPoseInWorld(modelName,Vector3d(x,y,z));
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
    scene = ui->widget->getScene();
    std::string robotprefix = "robot_";
    std::string ballprefix = "ball_";
    std::string bsballname = "ball_bs";

    bsBallVisual = scene->GetVisual(bsballname);
    if(bsBallVisual!=NULL){
        bsBallVisual = bsBallVisual->GetRootVisual();
        disconnect(ui->widget,SIGNAL(newFrameRendered()),this,SLOT(setup3DVisualPtrs()));
        sendDataTimer->start(CYCLE_TIME);
    }else return;

    for(int i=0;i<NROBOTS;i++){
        robotVisuals[i] = scene->GetVisual(robotprefix+std::to_string(i+1))->GetRootVisual();
        ballVisuals[i] = scene->GetVisual(ballprefix+std::to_string(i+1))->GetRootVisual();
    }
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    ui->widget->close();
    run_gz->terminate();
    int status = 0;
    waitpid(run_gz->get_id(), &status, WNOHANG);
    event->accept();
}

template<typename Message>
void MainWindow::deserializeROSMessage(udp_packet *packet, Message *msg)
{
    ros::serialization::IStream istream(packet->packet, packet->packet_size);
    ros::serialization::deserialize(istream, *msg);
}

MainWindow::~MainWindow()
{
    delete ui;
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
