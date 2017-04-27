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

namespace gazebo
{
  /// \brief A plugin to control a soccer robot's omnidirectional movement, kicker and dribbler
  class Bs_RPlugin : public ModelPlugin
  {
    public: 
    
    /// \brief Constructor. Initialized deafult variables for various variables
    Bs_RPlugin();

    /// \brief Destructor
    virtual ~Bs_RPlugin();
    
    /// \brief Plugin Load function. Initializes all ros topics for the robot model,
    /// also starting message queue thread. Connects gazebo events like world update,
    /// time reset and world reset
    /// \param _parent - Model pointer to the model defining this plugin
    /// \param _sdf - pointer to the SDF of the model
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
    
    private:
    
    /// \brief gets a list of models in the world and renames itself, accordingly to the
    /// existing robots already spawned.
    void autoRenameRobot();
    
    /// \brief called by event signal when a model or server reset happens
    void onReset();
    
    // VARIABLES
        
    /// \brief Pointer to the model that defines this plugin
    physics::ModelPtr _model_;
    
    /// \brief Model unique ID
    unsigned int model_id_;
    
    /// \brief Robot unique ID inside team structure ranging from 1~6
    unsigned int team_id_;

    /// \brief Transport node used to communicate with the transport system
    transport::NodePtr _node_;

    /// \brief Starting pose of the robot
    math::Pose initial_pose_;
    
    /// \brief pointer to server update event
    event::ConnectionPtr _reset_connection_;
    event::ConnectionPtr _timeres_connection_;
  };
}
#endif
