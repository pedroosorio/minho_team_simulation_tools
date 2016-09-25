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

#include "minho_world.hh"

using namespace gazebo;
Minho_World *me;
// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(Minho_World);

/////////////////////////////////////////////////
Minho_World::Minho_World()
{
    models_.clear();
    model_config_.clear();
    me = this;
}

Minho_World::~Minho_World()
{
    for(unsigned int i=0;i<model_config_.size();i++) free(model_config_[i]);
}
/////////////////////////////////////////////////
void Minho_World::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
{
    ROS_INFO("Starting world plugin ...");
    node_ = transport::NodePtr(new transport::Node());
    world_ = _parent;
    node_->Init(_parent->GetName());
    initPluginConfiguration(_sdf);
    world_mutex_ = _parent->GetSetWorldPoseMutex();
    //Init subsciber and publisher
    factory_pub_ = node_->Advertise<gazebo::msgs::Factory>("/gazebo/default/factory");
    cmd_sub_ = node_->Subscribe("/gazebo/default/factorybridge", &Minho_World::parseCommand);
}


void Minho_World::initPluginConfiguration(sdf::ElementPtr _sdf)
{    
    // count number of models
    unsigned int defined_models = 0;
    sdf::ElementPtr model;
    model = _sdf->GetFirstElement();
    while(model){
        if(model->HasAttribute("model_id")){ // valid model
            defined_models++;
        }
        model = model->GetNextElement();
    }
    ROS_INFO("Found %d defined models in world plugin.",defined_models);
    
    // actually load models
    int model_id = 0;
    model = _sdf->GetFirstElement();
    while(model){
        if(model->HasAttribute("model_id")){ // valid model
            if(model->GetAttribute("model_id")->Get(model_id)){
                AvailableModelPtr mod = new AvailableModel;
                mod->id = model_id;
                mod->path_to = model->GetValue()->GetAsString();
                model_config_.push_back(mod);
            } else {
                ROS_WARN("Model definition error in %s",model->GetValue()->GetAsString().c_str());
            }
        }
        model = model->GetNextElement();
    }
}

void Minho_World::spawnModel(unsigned int id, std::string default_name)
{
    bool pauseState = world_->IsPaused();
    world_->SetPaused(true);

    for(unsigned int i=0;i<model_config_.size();i++){
        if(model_config_[i]->id == id){              
            gazebo::msgs::Factory new_model;
            if(default_name.compare("")!=0){
                std::string filename = common::ModelDatabase::Instance()->GetModelFile(
                model_config_[i]->path_to);
        
                std::ifstream sdf_file;
                sdf_file.open(filename);
                std::string line;
                std::string sdf_data = "";
                if(sdf_file.is_open()){
                    while(getline(sdf_file,line)){sdf_data += line; sdf_data +="\n";}
                } else {
                    ROS_ERROR("Cannot open file for %s",model_config_[i]->path_to.c_str());
                }
                
                std::size_t found = sdf_data.find("model name=",0);
                if (found!=std::string::npos){
                    std::size_t found_end = sdf_data.find("\"",static_cast<int>(found)+13);
                    std::string sub_name = "model name=\"";
                    sub_name += default_name;
                    sub_name += "\"";
                    sdf_data.replace(static_cast<int>(found),static_cast<int>(found_end)-static_cast<int>(found)+1
                    ,sub_name);
                    new_model.set_sdf(sdf_data);
                } else new_model.set_sdf_filename(model_config_[i]->path_to);
            } else new_model.set_sdf_filename(model_config_[i]->path_to);
            // Pose to initialize the model to
            msgs::Set(new_model.mutable_pose(),
                      ignition::math::Pose3d(
                      ignition::math::Vector3d(5, 0, 0),
                      ignition::math::Quaterniond(0, 0, 0))
                      );
            factory_pub_->Publish(new_model);
            ROS_INFO("\nSpawned model '%s'",default_name.c_str());
            break;
        } 
    }
    
    world_->SetPaused(pauseState);
}

void Minho_World::deleteModel(std::string name)
{
    bool model_deleted = false;
    models_ = world_->GetModels();   // get models

    for(unsigned int i = 0; i<models_.size(); i++){
        if(name.compare(models_[i]->GetName())==0){
            ROS_INFO("Found model to be deleted.");
            //world_->RemoveModel(models_[i]->GetName()); 
            model_deleted = true;   
        }
    }
    
    if(model_deleted) ROS_INFO("\nDeleted model '%s'",name.c_str());
    else ROS_INFO("\nFailed to delete model '%s'",name.c_str());
    std::stringstream models_list;
    models_list << "Current Models in World:";
    for(unsigned int m = 0;m<models_.size();m++) models_list << "\n\t\t\t\tModel " << m << " -> " << models_[m]->GetName();
    models_list << "\n";
    ROS_INFO("%s",models_list.str().c_str());
}

void Minho_World::parseCommand(ConstGzStringPtr &_msg)
{
    std::string command = _msg->data();
    std::string action = "";
    std::string value = "";
    size_t pos = 0;
    if((pos = command.find(":")) != std::string::npos) {
        action = command.substr(0, pos);
        command.erase(0, pos + 1);
    }
    
    if(action.compare("spawn")==0){
        size_t pos_2 = 0;
        if((pos_2 = command.find(",")) != std::string::npos) {
            value = command.substr(0, pos_2);
            command.erase(0, pos_2 + 1);
            int type = std::atoi(value.c_str());
            ROS_INFO("Spawning model '%s' as type '%d'",command.c_str(),type);
            me->spawnModel(type,command);
        }     
    
    } else if(action.compare("remove")==0){
        ROS_INFO("Removing model '%s'",command.c_str());
        me->deleteModel(command);
    } else {
        ROS_ERROR("Wrong model managing command : '%s'",_msg->data().c_str());
    }
}
