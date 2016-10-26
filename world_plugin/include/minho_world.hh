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
#include <sdf/Param.hh>
#include <sdf/sdf.hh>
#include "ros/ros.h"
#include <cstdlib>
#include <fstream>
#define DEG_TO_RAD M_PI/180.0
#define RAD_TO_DEG 180.0/M_PI

typedef struct AvailableModel{
    int id;
    std::string path_to;
} AvailableModel;
typedef struct AvailableModel* AvailableModelPtr;

namespace gazebo
{
  /// \brief A plugin to control a soccer robot's omnidirectional movement, kicker and dribbler
  class Minho_World : public WorldPlugin
  {
    public: 
    /// \brief Constructor
    Minho_World();

    /// \brief Destructor
    virtual ~Minho_World();
    
    /// \brief Plugin Load function
    /// \param _parent - Model pointer to the model defining this plugin
    /// \param _sdf - pointer to the SDF of the model
    void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);
    
    /// \brief initializes the parameters in the plugin's configuration,
    /// such as available models
    /// \param _sdf - configurations for the plugin in sdf file
    void initPluginConfiguration(sdf::ElementPtr _sdf);
    
    /// \brief spawns a model into a world, given its id, in relation with 
    /// model_config_ models, defined in plugin's sdf.
    /// \param id - id of the type of model to be spawned (matchin) plugin configuration
    /// \param default_name - name of the new model
    void spawnModel(unsigned int id, std::string default_name);
    
    /// \brief deletes a model with a given name.
    /// \param name - name of the model to be deleted
    void deleteModel(std::string name);
    
    /// \brief receives messages in string form to add or remove a model.
    /// \param _msg - data received in callback, containing info about
    /// spawning or removing a model
    static void parseCommand(ConstGzStringPtr &_msg);
    
    private:
    // VARIABLES
    /// \brief Transport node used to communicate with the transport system
    transport::NodePtr node_;
    /// \brief pointer to the world object of the simulation
    physics::WorldPtr world_;   
    /// \brief vector to hold the current models present in the world
    std::vector<physics::ModelPtr> models_;
    ///  \brief vector of structs to hold model configuration to be able
    /// to configure and easily change the models that are available for 
    /// the world.
    std::vector<AvailableModelPtr> model_config_;
    /// \brief pointer to publish over factory topic, to add models to 
    /// the simulation
    gazebo::transport::PublisherPtr factory_pub_;
    /// \brief pointer to subscriver over gz_string topic, to receive
    /// add or delete comands for models in the simulation.
    gazebo::transport::SubscriberPtr cmd_sub_;
    /// \brief mutex to synchronize threads when adding or removing models 
    /// from world, increasing thread-safety.
    boost::mutex *world_mutex_;
  };
}
#endif
