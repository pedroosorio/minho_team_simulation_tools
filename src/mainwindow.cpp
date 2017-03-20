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
    ui->widget->init("mtbasestation");
    ui->widget->setGrid(false);
    ui->widget->setAllControlsMode(true);
    mBsInfo.agent_id = 6;
    run_gz = NULL;
    for(int i=0;i<NROBOTS;i++) { robotState[i] = false; robotReceivedPackets[i]=0; }

    robotStateDetector = new QTimer();
    connect(robotStateDetector,SIGNAL(timeout()),this,SLOT(detectRobotsState()));
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

    if(agent_data.agent_id<1 || agent_data.agent_id>=(NROBOTS+1));
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
    for(int i=0;i<NROBOTS;i++){
        if(robotState[i]){
            // show stuff from robot i
        } else {
            // hide stuff from robot i
        }
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

void MainWindow::detectRobotsState()
{
    for(int i=0;i<NROBOTS;i++) {
        if(robotReceivedPackets[i]>8){
            robotState[i] = true;
            robotReceivedPackets[i] = 0;
        } else { robotReceivedPackets[i] = false; }
    }
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    ui->widget->close();
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
