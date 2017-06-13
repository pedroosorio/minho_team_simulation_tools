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

#include "bs_rplugin.hh"
#define Y_AXIS_MULTIPLIER -1.0
using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(Bs_RPlugin);

/// \brief Constructor. Initialized deafult variables for various variables
Bs_RPlugin::Bs_RPlugin()
{
    initial_pose_ = math::Pose(0.0,-6.4*(-Y_AXIS_MULTIPLIER),0.0,0.0,0.0,0.0);
    team_id_ = 0; //null
}

/// \brief Destructor
Bs_RPlugin::~Bs_RPlugin()
{

}

/// \brief Plugin Load function. Initializes all ros topics for the robot model,
/// also starting message queue thread. Connects gazebo events like world update,
/// time reset and world reset
/// \param _parent - Model pointer to the model defining this plugin
/// \param _sdf - pointer to the SDF of the model
void Bs_RPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    // Rename model to specification
    _model_ = _parent;
    _node_ = transport::NodePtr(new transport::Node());
    _node_->Init(_model_->GetWorld()->GetName());
    model_id_ = _model_->GetId();
    // Stop simulation to prevent odd behaviors
    gazebo::physics::WorldPtr world = _model_->GetWorld();

    bool pauseState = world->IsPaused();
    world->SetPaused(true);
    world->DisableAllModels();
    world->EnablePhysicsEngine(false);

    autoRenameRobot();
    _reset_connection_ = event::Events::ConnectTimeReset(
    boost::bind(&Bs_RPlugin::onReset, this));
    _timeres_connection_ = event::Events::ConnectWorldReset(
    boost::bind(&Bs_RPlugin::onReset, this));
    
    world->EnablePhysicsEngine(true);
    world->EnableAllModels();
    world->SetPaused(pauseState);
    
    //Place the robot in the initial position regarding its team_id_
    initial_pose_.pos.x = 0.8*(float)team_id_;
    _model_->SetWorldPose(initial_pose_);
}

/// \brief gets a list of models in the world and renames itself, accordingly to the
 /// existing robots already spawned.
void Bs_RPlugin::autoRenameRobot()
{
    team_id_ = 1;
    std::string tid = _model_->GetName(); 
    char id = tid[tid.length()-1];
    team_id_ = id-'0';
  
    // Auto numbering
    std::string pref = "asd";
    sdf::ElementPtr original_sdf = _model_->GetSDF();
    sdf::ElementPtr root_link;
    sdf::ElementPtr numbering_back, numbering_left, numbering_right;
    sdf::ElementPtr number_uri;
    bool root_link_found = false;
    root_link = original_sdf->GetElement("link");
    while(!root_link_found && root_link){
        if(root_link->GetAttribute("name")->GetAsString().compare("numberings")==0){
            root_link_found = true;    
        } else root_link = root_link->GetNextElement();
    }
     
    // numbering back
    bool back_visual_found = false;
    numbering_back = root_link->GetElement("visual");
    while(!back_visual_found && numbering_back){
        if(numbering_back->HasAttribute("name") && (numbering_back->GetAttribute("name")->GetAsString().compare("numbering_back")==0)){
            back_visual_found = true;    
        } else numbering_back = numbering_back->GetNextElement();
    }
    
    // Apply correct stl matching team_id_
    if(back_visual_found) {
        number_uri = numbering_back->GetElement("geometry")->GetElement("mesh")->GetElement("uri");
        std::stringstream new_uri; new_uri << "model://bs_robot/meshes/nr_" << std::to_string(team_id_)<< ".stl";
        number_uri->GetValue()->SetFromString(new_uri.str());
    }
   
    // numbering left
    bool left_visual_found = false;
    numbering_left = root_link->GetElement("visual");
    while(!left_visual_found && numbering_left){
        if(numbering_left->HasAttribute("name") && (numbering_left->GetAttribute("name")->GetAsString().compare("numbering_left")==0)){
            left_visual_found = true;    
        } else numbering_left = numbering_left->GetNextElement();
    }
    
    // Apply correct stl matching team_id_
    if(left_visual_found) {
        number_uri = numbering_left->GetElement("geometry")->GetElement("mesh")->GetElement("uri");
        std::stringstream new_uri; new_uri << "model://bs_robot/meshes/nr_" << std::to_string(team_id_)<< ".stl";
        number_uri->GetValue()->SetFromString(new_uri.str());
    }
    
    // numbering right
    bool right_visual_found = false;
    numbering_right = root_link->GetElement("visual");
    while(!right_visual_found && numbering_right){
        if(numbering_right->HasAttribute("name") && (numbering_right->GetAttribute("name")->GetAsString().compare("numbering_right")==0)){
            right_visual_found = true;    
        } else numbering_right = numbering_right->GetNextElement();
    }
    
    // Apply correct stl matching team_id_
    if(right_visual_found) {
        number_uri = numbering_right->GetElement("geometry")->GetElement("mesh")->GetElement("uri");
        std::stringstream new_uri; new_uri << "model://bs_robot/meshes/nr_" << std::to_string(team_id_)<< ".stl";
        number_uri->GetValue()->SetFromString(new_uri.str());
    }
    
    // Update model's SDF
    _model_->Update();
}

/// \brief called by event signal when a model or server reset happens
void Bs_RPlugin::onReset()
{
   _model_->SetWorldPose(initial_pose_);  
}

   
