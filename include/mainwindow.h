#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "multicastpp.h"
#include <QCloseEvent>
#include <QTimer>
#include <ros/ros.h>
#include <minho_team_ros/interAgentInfo.h> // msg_id = 1;
#include <minho_team_ros/robotInfo.h>
#include <minho_team_ros/hardwareInfo.h>
#include <minho_team_ros/baseStationInfo.h> // msg_id = 0;
#include <std_msgs/UInt8.h>
#include <boost/filesystem.hpp>
#include <boost/process.hpp>
#include <boost/process/child.hpp>

using minho_team_ros::interAgentInfo;
using minho_team_ros::robotInfo;
using minho_team_ros::hardwareInfo;
using minho_team_ros::baseStationInfo;
using std_msgs::UInt8;

#define NROBOTS 5

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(bool isOfficialField, Multicastpp *coms, QWidget *parent = 0);
    ~MainWindow();
    /// \brief function that parses upd packet into interAgentInfo message or baseStationInfo message.
    void updateAgentInfo(void *packet);
    /// \brief close event override
    void closeEvent(QCloseEvent *event);
private:
    /// \brief function that reads msg_id parameter in packet and returns true if msg_id = 1 (is interAgentInfo)
    bool isAgentInfoMessage(udp_packet *packet);
    /// \brief templated function to deserialize udp_packet into ROS message
    template<typename Message>
    void deserializeROSMessage(udp_packet *packet, Message *msg);
    /// \brief slot function to compute and send current baseStationInfo message
    void sendBaseStationUpdate();
    /// \brief function that serializes baseStationInfo into UDP packet
    template<typename Message>
    void serializeROSMessage(Message *msg, uint8_t **packet, uint32_t *packet_size);
    /// \brief function that sends information over multicast socket
    void sendInfoOverMulticast();
    /// \brief creates a system process to hold gazebo for base station
    void initGazeboBaseStationWorld();
    /// \brief init gazebo for base station
    bool runGzServer();
private slots:
    /// \brief function that detects if a robot is online or offline based on received packets
    void detectRobotsState();
signals:
    void newRobotInformationReceived();
private:
    Ui::MainWindow *ui;
    /// \brief vector that holds most recent info from a robot
    interAgentInfo robots[NROBOTS];
    /// \brief vector that holds if a robot is online or offline
    bool robotState[NROBOTS];
    /// \brief vector to hold number of received packets by a certain agent
    int robotReceivedPackets[NROBOTS];
    /// \brief timer that calls a function to detect if an agent is online or offline
    QTimer *robotStateDetector;
    /// \brief pointer to multicast class, to send info
    Multicastpp *rtdb;
    /// \brief buffer to use in message serialization
    boost::shared_array<uint8_t> serialization_buffer;
    /// \brief object that holds info to be sent to robots
    baseStationInfo mBsInfo;
    /// \brief variable that tells if used field is oficial or lar
    bool isOfficialField;
    /// \brief process handler for gazebo
    boost::process::child *run_gz;

};

#endif // MAINWINDOW_H
