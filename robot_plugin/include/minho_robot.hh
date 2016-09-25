#ifndef _GAZEBO_MINHO_HARDWARE_HH_
#define _GAZEBO_MINHO_HARDWARE_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/Events.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
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
#include <sdf/Param.hh>
#include <boost/thread/thread_time.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/mutex.hpp>

#define DEG_TO_RAD M_PI/180.0
#define RAD_TO_DEG 180.0/M_PI

//ROS includes
#include "ros/ros.h"
#include "minho_team_ros/robotInfo.h"
#include "minho_team_ros/controlInfo.h"
#include "minho_team_ros/teleop.h"
#include "minho_team_ros/position.h"
using namespace ros;
using minho_team_ros::robotInfo; //Namespace for robot information msg - PUBLISHING
using minho_team_ros::controlInfo; //Namespace for control information msg - SUBSCRIBING
using minho_team_ros::teleop; //Namespace for teleop information msg - SUBSCRIBING

namespace gazebo
{
  /// \brief A plugin to control a soccer robot's omnidirectional movement, kicker and dribbler
  class Minho_Robot : public ModelPlugin
  {
    public: 
    
    /// \brief Constructor
    Minho_Robot();

    /// \brief Destructor
    virtual ~Minho_Robot();
    /// \brief Plugin Load function
    /// \param[in] _parent Model pointer to the model defining this plugin
    /// \param[in] _sdf pointer to the SDF of the model
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Applies the desired velocities to the robot, given
    /// the linear velocity, direction of movement and angular velocity
    /// \param[in] command has 3 components: (1) is linear velocity [0~100]
    /// (2) is direction of movement [0~2*pi] and (3) is angular velocity [-100~100]
    void applyVelocities(math::Vector3 command);
    
    private:
    
    /// \brief maps a velocity of the maximum velocity given a percentage.
    /// \param[in] percentage percentage of the maximum velocity to be applied
    /// \param[in] limit max velocity to be applied
    double mapVelocity(double percentage, double limit);
    
    /// \brief gets a list of models in the world and renames itself, accordingly to the
    /// existing robots already spawned.
    void autoRenameRobot();
    
    /// \brief called by event signal every server simulation iteration. Used to reset parameters
    /// like velocities and others.
    void onUpdate();
    
    /// \brief called by event signal when a model or server reset happens.
    void onReset();
    
    /// \brief callback to receive ROS messages published over the matching ros topic, in order
    /// to retrieve data about comands for robot motion.
    void controlInfoCallback(const controlInfo::ConstPtr& msg);
    
    /// \brief callback to receive ROS messages published over the matching ros topic, in order
    /// to retrieve data about comands for robot motion.
    void teleopCallback(const teleop::ConstPtr& msg);
    
    /// \brief thread to queue incoming data to callback queue, that subsequently calls the respective
    /// callback, passing the matching data to it.
    void message_queue_thread();
    
    /// \brief searches for game ball inside the world and stores a pointer to the model
    void getGameBallModel();
    
    /// \brief runs ball detection "sensor", using ball and robot pose's, computing the distance between
    /// them, comparing with a threshold.
    void detectBallPossession();
    
    /// \brief creates a ROS message, updates all the information and sends it through the publisher
    void publishRobotInfo();
    
    /// \brief dribbles the ball, given the velocity vector of the robot
    void dribbleGameBall();
    
    /// \brief kicks the ball, through the floor (passing) or the air (shoothing)
    /// allowing a slight variation in the direction of kicking
    void kickGameBall(bool pass, int strength, int direction);
    
    /// \brief reads and parses the elements passed to the plugin inside the model's sdf
    /// and initializes the matching variables
    void initializePluginParameters(sdf::ElementPtr _sdf);
    
    /// \brief generates random noise to add gaussian noise to a variable read from the world
    /// like ball position and obstacles position
    double generateNoise(double mean, double stdev, double min, double max);
    
    /// \brief detects obstacles in the view range, whether they being friendly or foe,
    /// at this point, in reality, the robot doesn't distinguish between friends or foes
    std::vector<minho_team_ros::position>detectObstacles();
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
    
    /// \brief pointer to world's set pose mutex. This pointer will make changes to the
    /// models become more thread safe.
    boost::mutex *world_mutex_;
   
  };
}
#endif
