#ifndef _GAZEBO_MINHO_HARDWARE_HH_
#define _GAZEBO_MINHO_HARDWARE_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <ignition/math.hh>
#include <iostream>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>

namespace gazebo
{
  /// \brief A plugin to control a soccer robot's and ball pose in world to provide 3D Basestation
  class Bs_Plugin : public WorldPlugin
  {
    public: 
    /// \brief Constructor
    Bs_Plugin();

    /// \brief Destructor
    virtual ~Bs_Plugin();
    
    /// \brief Plugin Load function
    /// \param _parent - Model pointer to the model defining this plugin
    /// \param _sdf - pointer to the SDF of the model
    void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);
    /// \brief callback for modify_sub subscriber
    /// \param _msg - message received by subscriber
    void modifyModel(ConstModelPtr &_msg);
    /// \brief rotates a point
    math::Pose rotate_pose(math::Pose pose, float yawrotation);
    private:
    // VARIABLES
    /// \brief Transport node used to communicate with the transport system
    transport::NodePtr node_;
    /// \brief pointer to the world object of the simulation
    physics::WorldPtr world_;   
    /// \brief pointer to subscriber over model topic, to receive
    /// info on how to move a certain model
    gazebo::transport::SubscriberPtr modify_sub;
    /// \brief mutex to synchronize threads when adding or removing models 
    /// from world, increasing thread-safety.
    boost::mutex *world_mutex_;

    float velocity,last_velocity;
  };
}
#endif
