#ifndef WORLDMANAGER_H
#define WORLDMANAGER_H

///
/// \brief This code is part of the MinhoTeam Simulation Tools, developed by Pedro
/// Osório Silva (pedroosorio.eeic@gmail.com) and it's free to distribute and edit.
///

#include <QObject>
#include <QMessageBox>
#include "gazebo/gazebo.hh"
#include "gazebo/common/common.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/transport/Publisher.hh"
#include "gazebo/transport/Subscriber.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/msgs/world_control.pb.h"
#include "gazebo/msgs/world_stats.pb.h"
#include "gazebo/msgs/gz_string.pb.h"
#include "gazebo/msgs/time.pb.h"
#include "gazebo/msgs/light.pb.h"
#include "gazebo/gazebo_client.hh"
#include <QDebug>
#include <functional>

class WorldManager : public QObject
{
    Q_OBJECT
public:
    ///
    /// \brief WorldManager implements the control of the simulation, using Gazebo's transport layer,
    /// to control the simulation flow, display simulation information and spaw/delete, move, tele-operate
    /// any non-static model in the simulation.
    /// \param parent widget
    ///
    explicit WorldManager(QObject *parent = 0);

    ///
    /// \brief WorldManager::~WorldManager Destructor for the class. Finishes the node and transport layer
    /// services.
    ///
    ~WorldManager();
signals:

public slots:
    // Flow Control of the simulation
    ///
    /// \brief pause pauses the simulation, sending the control command through Gazebo's transport layer.
    ///
    void pause();

    ///
    /// \brief unpause unpauses the simulation, sending the control command through Gazebo's transport layer.
    ///
    void unpause();

    ///
    /// \brief resetSimulation resets all the models to their default position, including the simulation
    /// time itself.
    ///
    void resetSimulation();

    ///
    /// \brief resetModels resets all the models to their default position.
    ///
    void resetModels();

    // Getters of simulation state
    ///
    /// \brief getState returns the state of the simulation as a QString.
    /// \return - Returns "Paused" is simulation is paused and "Running" if simulation is unpaused.
    ///
    QString getState();

    ///
    /// \brief isPaused returns the state of the simulation as a boolean.
    /// \return - Returns true is simulation is paused and false if simulation is unpaused.
    ///
    bool isPaused();

    ///
    /// \brief getSimTime returns the simulation time as a QString.
    /// \return - simulation time (HH:MM:SS::MS) in a QString.
    ///
    QString getSimTime();

    ///
    /// \brief getRealTime returns the real time as a QString.
    /// \return - real time (HH:MM:SS::MS) in a QString.
    ///
    QString getRealTime();

    ///
    /// \brief addModel adds a model, if not existing, to the simulation.
    /// \param type type of the model, defined in world.sdf plugin parameters.
    /// \param name name of the model.
    ///
    void addModel(int type, QString name);

    ///
    /// \brief removeModel removes a model form the simulation.
    /// \param name of the model to be removed.
    ///
    void removeModel(QString name);

    ///
    /// \brief isModelInWorld detects if a specific model named "name" is already
    /// spawned.
    /// \param name name of the model to look for.
    ///
    bool isModelInWorld(QString name);
    ///
    /// \brief getModels returns the existing models in the world.
    /// \return list of model's poses.
    ///
    std::vector<gazebo::msgs::Pose> getModels();
private slots:

    ///
    /// \brief throwConnectionError throws a message and quits the simulator if a critical error is
    /// encountered.
    ///
    void throwConnectionError();

    // Callbacks for Gazebo
    ///
    /// \brief world_stats_callback callback to receive simulation state's updates, published by
    /// gzserver over the Gazebo's transport layer.
    /// \param _msg
    ///
    void world_stats_callback(ConstWorldStatisticsPtr &_msg);
    ///
    /// \brief model_pose_callback callback to receive model pose information.
    /// \param _msg
    ///
    void model_pose_callback(ConstPosesStampedPtr &_msg);
private:
    // Variables
    // *******************************************************************************************
    // *******************************************************************************************
    // Gazebo Publish/Subscribe Variables
    ///
    /// \brief _ptr_ Node pointer to provide acess to Gazebo's transport layer and publish/subscribe
    /// system.
    ///
    gazebo::transport::NodePtr _ptr_;
    ///
    /// \brief _world_pub_ Publisher of world control information : Pause/Unpause, Reset.
    ///
    gazebo::transport::PublisherPtr _world_pub_;
    ///
    /// \brief _world_sub_ Subscriber for simulation state : SimTime, RealTime and State.
    ///
    gazebo::transport::SubscriberPtr _world_sub_;
    ///
    /// \brief _resetworld_ Reset allocated object that defines the reset state of a
    /// world control message sent by _world_pub_ over the matching topic.
    ///
    gazebo::msgs::WorldReset *_resetworld_;

    // World Statistics
    ///
    /// \brief paused_ Boolean representation of the state of the simulation. True if the simulation
    /// is paused and false if the simulation is running.
    ///
    bool paused_;
    ///
    /// \brief sim_time_ QString representation of the current time of the simulation received by
    /// _world_sub_.
    /// \brief real_time_ QString representation of the current real time in simulation received by
    /// _world_sub_.
    ///
    QString sim_time_, real_time_;
    ///
    /// \brief Same as above but in a time object representation.
    ///
    gazebo::common::Time simTime_, realTime_;
    ///
    /// \brief manage_pub_ publisher to publish data to add or remove models from simulation.
    ///
    gazebo::transport::PublisherPtr manage_pub_;
    ///
    /// \brief modelsList holds the models list names of the world.
    ///
    std::vector<QString> modelsList;
    ///
    /// \brief model_list_sub_ Subscriber for model state string, to receive spawned model names.
    ///
    gazebo::transport::SubscriberPtr model_list_sub_;
    ///
    /// \brief models vector to hold pose information about models in the world.
    ///
    std::vector<gazebo::msgs::Pose> models;
signals:
    ///
    /// \brief new_world_stats signal feeded to the GUI (mainwindow) to display simulation information
    /// when it arrives, allowing asynchronous operation.
    /// \param state - State of the simulation. Please refer to getState();
    /// \param sim_time - Simulation time of the simulation. Please refer to getSimTime();
    /// \param real_time - Real time of the simulation. Please refer to getRealTime();
    ///
    void new_world_stats(QString state,QString sim_time, QString real_time);
    ///
    /// \brief new_poses signals that new poses have been received so the GUI can display them.
    /// \param poses is a vector of poses of the models present in the world.
    ///
    void new_poses(std::vector<gazebo::msgs::Pose> poses);
};

#endif // WORLDMANAGER_H
