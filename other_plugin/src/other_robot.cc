/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include "other_robot.hh"
#define Y_AXIS_MULTIPLIER -1.0
using namespace gazebo;


//Functions

//GetWorldPose ()
//GetId ()
//GetName ()
//SetAngularVel (const math::Vector3 &_vel)
//SetLinearVel (const math::Vector3 &_vel)
//SetWorldPose (const math::Pose &_pose, bool _notify=true, bool _publish=true)

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(Other_Robot);

/// \brief Constructor. Initialized deafult variables for various variables
Other_Robot::Other_Robot()
{

}

/// \brief Destructor
Other_Robot::~Other_Robot()
{

}

/// \brief Plugin Load function. Initializes all ros topics for the robot model,
/// also starting message queue thread. Connects gazebo events like world update,
/// time reset and world reset
/// \param _parent - Model pointer to the model defining this plugin
/// \param _sdf - pointer to the SDF of the model
void Other_Robot::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    // Rename model to specification
    _model_ = _parent;
    _node_ = transport::NodePtr(new transport::Node());
    _node_->Init(_model_->GetWorld()->GetName());

    // Stop simulation to prevent odd behaviors
    gazebo::physics::WorldPtr world = _model_->GetWorld();
    int team_id_ = 0;
    ROS_INFO("ModelName: %s",_model_->GetName().c_str());
    if(_model_->GetName().compare("other_robot")==0)team_id_=0;
    else team_id_=std::stoi(_model_->GetName().substr(12,12))+1;
    ROS_INFO("Team ID: %d",team_id_);
  
    //Place the robot in the initial position regarding its team_id_
    initial_pose_.pos.x = -0.8*(float)team_id_;
    initial_pose_.pos.y = 6.4;
    
    _model_->SetWorldPose(initial_pose_);
}

