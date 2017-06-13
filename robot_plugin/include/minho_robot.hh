#ifndef _GAZEBO_MINHO_HARDWARE_HH_
#define _GAZEBO_MINHO_HARDWARE_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/sensors/sensors.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Angle.hh>
#include <ignition/math/Vector2.hh>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sstream>
#include <vector>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <ros/callback_queue.h> 
#include <ros/subscribe_options.h>
#include <ros/advertise_service_options.h>
#include <sdf/Param.hh>
#include <boost/thread/thread_time.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/process.hpp> 
#define DEG_TO_RAD M_PI/180.0
#define RAD_TO_DEG 180.0/M_PI

//ROS includes
#include "ros/ros.h"
#include "minho_team_ros/robotInfo.h"
#include "minho_team_ros/controlInfo.h"
#include "minho_team_ros/teleop.h"
#include "minho_team_ros/position.h"
#include "minho_team_ros/obstacle.h"
#include "minho_team_ros/requestKick.h"

using namespace ros;
using minho_team_ros::robotInfo; //Namespace for robot information msg - PUBLISHING
using minho_team_ros::controlInfo; //Namespace for control information msg - SUBSCRIBING
using minho_team_ros::teleop; //Namespace for teleop information msg - SUBSCRIBING
using minho_team_ros::requestKick; // Namespace for kicking service
using minho_team_ros::position;
using minho_team_ros::obstacle;

namespace gazebo
{
  /// \brief A plugin to control a soccer robot's omnidirectional movement, kicker and dribbler
  class Minho_Robot : public ModelPlugin
  {
    public: 
    
    /// \brief Constructor. Initialized deafult variables for various variables
    Minho_Robot();

    /// \brief Destructor
    virtual ~Minho_Robot();
    
    /// \brief Plugin Load function. Initializes all ros topics for the robot model,
    /// also starting message queue thread. Connects gazebo events like world update,
    /// time reset and world reset
    /// \param _parent - Model pointer to the model defining this plugin
    /// \param _sdf - pointer to the SDF of the model
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Applies the desired velocities to the robot, given
    /// the linear velocity, direction of movement and angular velocity
    /// \param command - has 3 components: (1) is linear velocity [0~100]
    /// (2) is direction of movement [0~360] and (3) is angular velocity [-100~100]
    void applyVelocities(math::Vector3 command);
    
    private:
    /// \brief maps a velocity of the maximum velocity given a percentage.
    /// \param percentage - percentage of the maximum velocity to be applied
    /// \param limit - max velocity to be applied
    /// \return mapped value/velocity given a limit and a percentage
    double mapVelocity(double percentage, double limit);
    
    /// \brief gets a list of models in the world and renames itself, accordingly to the
    /// existing robots already spawned.
    void autoRenameRobot();
    
    /// \brief called by event signal every server simulation iteration. Used to reset 
    /// parameters like velocities and others
    void onUpdate();
    
    /// \brief called by event signal when a model or server reset happens
    void onReset();
    
    /// \brief callback to receive ROS messages published over the matching ros topic,
    /// in order to retrieve data about comands for robot motion.
    void controlInfoCallback(const controlInfo::ConstPtr& msg);
    
    /// \brief callback to receive ROS messages published over the matching ros topic,
    /// in order to retrieve data about comands for robot motion.
    void teleopCallback(const teleop::ConstPtr& msg);
    
    /// \brief function to actuate kicker, in order to kick the ball. Only kicks if
    /// the robot detects that has the ball inside
    /// \param req - request data received in requestKick service
    /// \param res - response data, flaggin if the kick was taken or not
    bool kickServiceCallback(requestKick::Request &req,requestKick::Response &res);
    
    /// \brief thread to queue incoming data to callback queue, that subsequently calls 
    /// the respective callback, passing the matching data to it.
    void message_queue_thread();
    
    /// \brief searches for game ball inside the world and stores a pointer to the model
    void getGameBallModel();
    
    /// \brief runs ball detection "sensor", using ball and robot pose's, computing the
    /// distance between them, comparing with a threshold.
    void detectBallPossession();
    
    /// \brief creates a ROS message, updates all the information and sends it through the
    /// publisher
    void publishRobotInfo();
    
    /// \brief dribbles the ball, given the velocity vector of the robot
    void dribbleGameBall();
    
    /// \brief kicks the ball, through the floor (passing) or the air (shoothing)
    /// allowing a slight variation in the direction of kicking
    /// \param pass - define wether the kick is a pass or not
    /// \param strength - define the strength of the kick
    /// \param direction - define the direction of the kick
    void kickGameBall(bool pass, int strength, int direction);
    
    /// \brief reads and parses the elements passed to the plugin inside the model's sdf
    /// and initializes the matching variables
    /// \param _sdf - pointer to sdf element containing
    void initializePluginParameters(sdf::ElementPtr _sdf);
    
    /// \brief generates random noise to add gaussian noise to a variable read from the 
    /// world like ball position and obstacles position
    /// \param mean - mean of the error to be generated
    /// \param stdev - standard deviation of the error to 
    /// \param min - minimum value of the output value
    /// \param max - maximum value of the output value
    /// \return generated noise value
    double generateNoise(double mean, double stdev, double min, double max);
    
    /// \brief detects obstacles in the view range, whether they being friendly or foe,
    /// at this point, in reality, the robot doesn't distinguish between friends or foes
    /// return vector of positions containing the position of the detected obstalces
    std::vector<minho_team_ros::obstacle>detectObstacles();
    
    /// \brief mapps a detected point relative to the robot to the world position
    /// \param robot - position of the robot in the field, in meters
    /// \param robot_heading - heading of the robot in º
    /// \param dist - detected distance in meters
    /// \param theta - angle of the detected point in relation to world's 0º
    /// \return - position of the mapped point in meters
    position mapPointToWorld(position robot, float robot_heading, float dist, float theta);
    
    /// \brief computes both ball and robot velocities to send in robotInfo using
    /// a simple low pass filter approach to reduce noisy estimates
    void computeVelocities();
    
    /// \brief boots indicated ROS nodes to add functionalities to the model
    /// This nodes have to be specified in plugin's SDF correctly, naming the
    /// package, node names, flags and placed in boot order
    /// \param _sdf - sdf struct containing plugin's data, containing Robot parameters
    /// and ROS boot node info
    void bootROSNodes(sdf::ElementPtr _sdf);
    
    /// \brief asserts flag value for ros node booting procedure. It can translate values
    /// started with '$' with plugin run time variables
    /// \param value - initial parameter value to be asserted
    /// \return - string with the asserted parameter
    std::string assertFlagValue(std::string value);
    
    /// \brief setus up model sensors, performing detection and type identification
    void setupSensors();
    
    math::Vector3 getAccelDeccelVelocity();
    
    // VARIABLES
        
    /// \brief Pointer to the model that defines this plugin
    physics::ModelPtr _model_;
    
    /// \brief Pointer to the model of the game ball
    physics::ModelPtr _ball_;
    
    /// \brief Model unique ID
    unsigned int model_id_;
    
    /// \brief Robot unique ID inside team structure ranging from 1~6
    unsigned int team_id_;

    /// \brief Transport node used to communicate with the transport system
    transport::NodePtr _node_;

    /// \brief Starting pose of the robot
    math::Pose initial_pose_;
    
    /// \brief current velocities aplied to the model
    math::Vector3 linear_velocity_, angular_velocity_;
    
    /// \brief direction of movement of omnidirectional platform
    float linear_vel_,mov_direction_, angular_vel_;
    
    /// \brief to enable or disable ros capabilities given the initialization state
    bool is_ros_initialized_;
    
    /// \brief node handler for ROS communication with publishers and subscribers
    ros::NodeHandle *_node_ros_;
    
    /// \brief subscriber for ControlInfo messages
    ros::Subscriber control_info_sub_;
    
    /// \brief subscriber for Teleop state messages
    ros::Subscriber teleop_state_sub_;
    
    /// \brief publisher for robotInfo messages
    ros::Publisher robot_info_pub_;
    
    /// \brief pointer to server update event
    event::ConnectionPtr _update_connection_;
    event::ConnectionPtr _reset_connection_;
    event::ConnectionPtr _timeres_connection_;
    
    /// \brief custom callback queue to implement a custom spinner to ROS.
    ros::CallbackQueue message_queue_;     // Custom Callback Queue.
    
    /// \brief thread object for the running calback thread
    boost::thread message_callback_queue_thread_;
    
    /// \brief data mutex to implement thread safety
    boost::mutex control_info_mutex_;
    boost::mutex tele_op_mutex_;
    
    /// \brief that defines whether the model is being controlled by teleop or by
    /// autonomous commands
    bool teleop_active_;
    
    /// \brief defines whether the current robot has the ball or not, given its distance to the
    /// center of the ball.
    bool has_game_ball_;
    
    /// \brief defines whether there is a model in the parent world that is the game ball.
    bool game_ball_in_world_;
    
    /// \brief threshold distance to judge whether the robot has the ball in its possession or not.
    /// Given the fact that if the ball is inside de "mouth" the distance is much smaller that in any
    /// other situation, it's an easy thing to judge.
    double poss_threshold_distance_;
    
    /// \brief hold the euclidean distance from the model to the ball, to detect ball possession.
    double distance_to_ball_;
    
    /// \brief holds the current model pose, that is updated every iteration of the simulation
    /// server.
    math::Pose model_pose_;
    math::Pose ball_pose_;
    
    /// \brief flags the state of the dribbling system. If true, when the ball is in possession of a robot
    /// he will dribble the ball, keeping it closer to himself.
    bool dribblers_on_;
    
    /// \brief flags if a kick to the ball has been requested. If its true, it will atempt to kick the ball 
    /// once it has the ball in his possession.
    bool kick_requested_;
    
    /// \brief variables to control the kicking strength and direction
    int kick_force_, kick_dir_;
    bool kick_is_pass_;
    
    /// \brief variables to control aspects of the robot's behaviour, expressed in the sdf file of the robot
    /// under the plugin tag
    float MAX_LIN_VEL, MAX_ANG_VEL, MAX_BALL_VEL, SHOOT_ANGLE;
    std::string BALL_MODEL_NAME;
    float VISION_RANGE_RADIUS;
    float MAX_BACKWARDS_VEL;
    float GRIP_DECAY;
    float MAX_ROTATION_VEL;
    /// \brief pointer to world's set pose mutex. This pointer will make changes to the
    /// models become more thread safe.
    boost::mutex *world_mutex_;
    
    /// \brief kick service server to handle kick server calls
    ros::ServiceServer kick_service; // Service to relay the configuration
    
    /// \brief robotInfo state variables 
    robotInfo current_state, last_state, last_vel_state;
    
    /// \brief vector holding handles to child processes
    std::vector<boost::process::child> childs;
    
    /// \brief vector holding handles to sensors attached to the model
    std::vector<gazebo::sensors::SensorPtr> sensors_;
    
    /// \brief handler of obstacle_detector mock sensor
    gazebo::sensors::RaySensor *obstacle_detector;

   /// \brief flag to implement control timeout
    bool controlCommandsReceived;
    
    /// \brief flag to control timer between kicks
    unsigned int kick_stab_counter;
    
    /// \brief acceleration and decceleration constants
    float constAccel, constDeccel;
    
    math::Vector3 targetCommand, lastCommand, currentCommand;
   
  };
}
#endif
