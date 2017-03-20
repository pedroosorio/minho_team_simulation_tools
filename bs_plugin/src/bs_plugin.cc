#include "bs_plugin.hh"

// modelPub = node->Advertise<msgs::Model>("~/mtbasestation/modify");

using namespace gazebo;
// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(Bs_Plugin);

/// \brief Constructor
Bs_Plugin::Bs_Plugin()
{
    velocity = last_velocity = 0.0;
}

/// \brief deletes allocated memory
Bs_Plugin::~Bs_Plugin()
{
}

/// \brief Plugin Load function
/// \param _parent - Model pointer to the model defining this plugin
/// \param _sdf - pointer to the SDF of the model
void Bs_Plugin::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
{
    node_ = transport::NodePtr(new transport::Node());
    world_ = _parent;
    node_->Init(_parent->GetName());
    world_mutex_ = _parent->GetSetWorldPoseMutex();
    world_->EnablePhysicsEngine(false);
    world_->DisableAllModels();
    world_->SetPaused(true);

    //init subscribers
    modify_sub = node_->Subscribe("~/bsplugin/modify", &Bs_Plugin::modifyModel, this);
}

void Bs_Plugin::modifyModel(ConstModelPtr &_msg)
{
    std::string model_name = _msg->name();
    physics::ModelPtr model = world_->GetModel(model_name);

    if(model){
        if(_msg->id()==0){ // Relative rotation of a model over the z axis (Yaw)
            model->SetWorldPose(rotate_pose(model->GetWorldPose(),_msg->pose().position().z()));
        } else if(_msg->id()==1){ // Relative movement over XY plane
            gazebo::math::Pose motion;
            motion.Set(_msg->pose().position().x(),_msg->pose().position().y(),0,0,0,0);
            model->SetWorldPose(model->GetWorldPose()+motion);
            velocity = sqrt(motion.pos.x*motion.pos.x+motion.pos.y*motion.pos.y);
            velocity = 0.5*velocity+0.5*last_velocity;
            if(velocity>0.1) {model->SetLinearVel(math::Vector3(motion.pos.x*20,motion.pos.y*20,0));}
            else {model->SetAngularVel(math::Vector3(0,0,0)); model->SetLinearVel(math::Vector3(0,0,0));}
            last_velocity = velocity;
        }  else if(_msg->id()==2){ // Relative movement over Z axis
            gazebo::math::Pose motion;
            motion.Set(0,0,_msg->pose().position().z(),0,0,0);
            model->SetWorldPose(model->GetWorldPose()+motion);
        } /*else if(_msg->id()==4){ // Absolute pose
            model->SetWorldPose(rotate_pose(_msg->pose().position(),_msg->pose().position().z()));
        }*/
    }

    if(_msg->id()==3){ // reset world
        world_->Reset();
    }
}

math::Pose Bs_Plugin::rotate_pose(gazebo::math::Pose pose, float yawrotation)
{
    gazebo::math::Vector3 orientation = pose.rot.GetAsEuler();
    orientation.z+=yawrotation;
    return gazebo::math::Pose(pose.pos,orientation);
}
