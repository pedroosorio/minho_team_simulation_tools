#include "worldmanager.h"

///
/// \brief WorldManager implements the control of the simulation, using Gazebo's transport layer,
/// to control the simulation flow, display simulation information and spaw/delete, move, tele-operate
/// any non-static model in the simulation.
/// \param parent widget
///
WorldManager::WorldManager(QObject *parent) : QObject(parent)
{
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    _ptr_ = node;
    modelsList.clear();
    node->Init("default");
    _world_pub_ = node->Advertise<gazebo::msgs::WorldControl>("/gazebo/default/world_control");
    _world_sub_ = node->Subscribe("/gazebo/default/world_stats",&WorldManager::world_stats_callback,this);
    model_list_sub_ = node->Subscribe("/gazebo/default/pose/info",&WorldManager::model_pose_callback,this);
    manage_pub_ = node->Advertise<gazebo::msgs::GzString>("/gazebo/default/factorybridge");
    if(!_world_pub_->WaitForConnection(gazebo::common::Time(1,0))) throwConnectionError();
}

///
/// \brief WorldManager::~WorldManager Destructor for the class. Finishes the node and transport layer
/// services.
///
WorldManager::~WorldManager()
{
    _ptr_->Fini();
    gazebo::client::shutdown();
}

///
/// \brief pause pauses the simulation, sending the control command through Gazebo's transport layer.
///
void WorldManager::pause()
{
    gazebo::msgs::WorldControl worldcontrol;
    worldcontrol.set_pause(true);
    _world_pub_->Publish(worldcontrol);
}

///
/// \brief unpause unpauses the simulation, sending the control command through Gazebo's transport layer.
///
void WorldManager::unpause()
{
    gazebo::msgs::WorldControl worldcontrol;
    worldcontrol.set_pause(false);
    _world_pub_->Publish(worldcontrol);
}

///
/// \brief resetSimulation resets all the models to their default position, including the simulation
/// time itself.
///
void WorldManager::resetSimulation()
{
    gazebo::msgs::WorldControl worldcontrol;
    _resetworld_ = NULL;
    _resetworld_ = new gazebo::msgs::WorldReset();
    _resetworld_->set_all(true);
    worldcontrol.set_allocated_reset(_resetworld_);
    _world_pub_->Publish(worldcontrol,true);
}

///
/// \brief resetModels resets all the models to their default position.
///
void WorldManager::resetModels()
{
    gazebo::msgs::WorldControl worldcontrol;
    _resetworld_ = NULL;
    _resetworld_ = new gazebo::msgs::WorldReset();
    _resetworld_->set_all(false);
    _resetworld_->set_model_only(true);
    worldcontrol.set_allocated_reset(_resetworld_);
    _world_pub_->Publish(worldcontrol,true);
}

///
/// \brief getState returns the state of the simulation as a QString.
/// \return - Returns "Paused" is simulation is paused and "Running" if simulation is unpaused.
///
QString WorldManager::getState()
{
    if(paused_) return "Paused";
    else return "Running";
}

///
/// \brief isPaused returns the state of the simulation as a boolean.
/// \return - Returns true is simulation is paused and false if simulation is unpaused.
///
bool WorldManager::isPaused()
{
    return paused_;
}

///
/// \brief getSimTime returns the simulation time as a QString.
/// \return - simulation time (HH:MM:SS::MS) in a QString.
///
QString WorldManager::getSimTime()
{
    return sim_time_;
}

///
/// \brief getRealTime returns the real time as a QString.
/// \return - real time (HH:MM:SS::MS) in a QString.
///
QString WorldManager::getRealTime()
{
    return real_time_;
}

void WorldManager::addModel(int type, QString name)
{
    QString data = "spawn:"+QString::number(type)+QString(",")+name;
    gazebo::msgs::GzString msg;
    msg.set_data(data.toStdString().c_str());
    if(manage_pub_) manage_pub_->Publish(msg,true);
}

void WorldManager::removeModel(QString name)
{
    QString data = "remove:"+name;
    gazebo::msgs::GzString msg;
    msg.set_data(data.toStdString().c_str());
    if(manage_pub_) manage_pub_->Publish(msg,true);
}

bool WorldManager::isModelInWorld(QString name)
{
    for(unsigned int i = 0;i<modelsList.size();i++){
        if(name==modelsList[i]) return true;
    }

    return false;
}

std::vector<gazebo::msgs::Pose> WorldManager::getModels()
{
    return models;
}

///
/// \brief throwConnectionError throws a message and quits the simulator if a critical error is
/// encountered.
///
void WorldManager::throwConnectionError()
{
    QMessageBox asd;
    asd.setWindowTitle("Failed to connect to Gazebo Master !");
    QString text = QString("Please open a terminal and write ")+
    QString("$gzserver before running MinhoTeam Simulation Tools.");
    asd.setInformativeText(text);
    asd.setStandardButtons(QMessageBox::Ok);
    asd.setIconPixmap(QPixmap("://resources/images/critical.png"));
    asd.exec();
    exit(0);
}

///
/// \brief world_stats_callback callback to receive simulation state's updates, published by
/// gzserver over the Gazebo's transport layer.
/// \param _msg
///
void WorldManager::world_stats_callback(ConstWorldStatisticsPtr &_msg)
{
    paused_ = _msg->paused();
    simTime_.Set(_msg->sim_time().sec(),_msg->sim_time().nsec());
    realTime_.Set(_msg->real_time().sec(),_msg->real_time().nsec());
    sim_time_ = QString::fromStdString(simTime_.FormattedString(gazebo::common::Time::HOURS
                                                                ,gazebo::common::Time::MILLISECONDS));
    real_time_ = QString::fromStdString(realTime_.FormattedString(gazebo::common::Time::HOURS
                                                                  ,gazebo::common::Time::MILLISECONDS));
    emit new_world_stats(getState(),sim_time_,real_time_);
}

void WorldManager::model_pose_callback(ConstPosesStampedPtr &_msg)
{
    std::vector<QString> list;
    std::vector<gazebo::msgs::Pose> poses;
    gazebo::msgs::Pose pose;
    QString name;
    unsigned int list_size = _msg->pose_size();
    for(unsigned int i = 0; i<list_size;i++){
        pose = _msg->pose(i);
        name = QString::fromStdString(pose.name());
        if(!name.contains(":")){ // If it's not a model's link
            list.push_back(name);
            poses.push_back(pose);
        }
    }
    modelsList = list;
    models = poses;
    emit new_poses(models);
}
