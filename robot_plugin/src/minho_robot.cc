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

#include "minho_robot.hh"

using namespace gazebo;


//Functions

//GetWorldPose ()
//GetId ()
//GetName ()
//SetAngularVel (const math::Vector3 &_vel)
//SetLinearVel (const math::Vector3 &_vel)
//SetWorldPose (const math::Pose &_pose, bool _notify=true, bool _publish=true)

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(Minho_Robot);

/////////////////////////////////////////////////
Minho_Robot::Minho_Robot()
{
    initial_pose_ = math::Pose(0.0,-6.4,0.0,0.0,0.0,0.0);
    linear_velocity_ = angular_velocity_ = math::Vector3(0.0,0.0,0.0);
    team_id_ = 0; //null
    is_ros_initialized_ = false;
    teleop_active_ = false;
    game_ball_in_world_ = false;
    poss_threshold_distance_ = 0.3;
    kick_requested_ = false;
    MAX_LIN_VEL = 2.5; // Default values
    MAX_ANG_VEL = 15.0;
    MAX_BALL_VEL = 8.0;
    SHOOT_ANGLE = 0.0;
    BALL_MODEL_NAME = "RoboCup MSL Ball";
    VISION_RANGE_RADIUS = 5.0;
}

Minho_Robot::~Minho_Robot()
{
    event::Events::DisconnectWorldUpdateBegin(_update_connection_);
    // Removes all callbacks from the queue. Does not wait for calls currently in progress to finish. 
    message_queue_.clear();
    // Disable the queue, meaning any calls to addCallback() will have no effect. 
    message_queue_.disable();
    message_callback_queue_thread_.join();
    _node_ros_->shutdown();
    delete _node_ros_;
}
/////////////////////////////////////////////////
void Minho_Robot::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    // Rename model to specification
    _model_ = _parent;
    _node_ = transport::NodePtr(new transport::Node());
    _node_->Init(_model_->GetWorld()->GetName());
    model_id_ = _model_->GetId();
    initializePluginParameters(_sdf);
    autoRenameRobot();
    ROS_WARN("Loading Minho_Robot for '%s' ...", _model_->GetName().c_str());
    
    if (!ros::isInitialized()){
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin.");
        is_ros_initialized_ = false;
    } else is_ros_initialized_ = true;

    //Place the robot in the initial position regarding its team_id_
    initial_pose_.pos.x = 0.8*(float)team_id_;
    _model_->SetWorldPose(initial_pose_);

    if(!is_ros_initialized_){
        ROS_ERROR("Plugin ROS failed to start on '%s'. \n", _model_->GetName().c_str());
    } else {
        ROS_INFO("Plugin ROS successfuly initialized. \n");
        // Initialize ROS publishers and subscribers
        _node_ros_ = new ros::NodeHandle();

        std::stringstream control_topic;
        control_topic << "minho_gazebo_robot" << std::to_string(team_id_) << "/controlInfo";

        std::stringstream teleop_topic;
        teleop_topic << "minho_gazebo_robot" << std::to_string(team_id_) << "/teleop";

        ros::SubscribeOptions control_info = ros::SubscribeOptions::create<minho_team_ros::controlInfo>(
        control_topic.str(), 100, boost::bind( &Minho_Robot::controlInfoCallback,this,_1),
        ros::VoidPtr(), &this->message_queue_);
        control_info_sub_ = _node_ros_->subscribe(control_info);

        ros::SubscribeOptions teleop_info = ros::SubscribeOptions::create<minho_team_ros::teleop>(
        teleop_topic.str(), 100, boost::bind( &Minho_Robot::teleopCallback,this,_1),
        ros::VoidPtr(), &this->message_queue_);
        teleop_state_sub_ = _node_ros_->subscribe(teleop_info);

        std::stringstream robotinfo_topic;
        robotinfo_topic << "minho_gazebo_robot" << std::to_string(team_id_) << "/robotInfo";

        robot_info_pub_ = _node_ros_->advertise<minho_team_ros::robotInfo>(robotinfo_topic.str(),100);
        if(!robot_info_pub_) ROS_ERROR("Failed to init robotInfo publiser for '%s'.", _model_->GetName().c_str());

        // Custom Callback Queue Thread. Use threads to process message and service callback queue
        message_callback_queue_thread_ = boost::thread(boost::bind(&Minho_Robot::message_queue_thread,this));
        ROS_WARN("Plugin ROS started on '%s'.", _model_->GetName().c_str());
    }

    // Connect server update callback  
    _update_connection_ = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&Minho_Robot::onUpdate, this));
    _reset_connection_ = event::Events::ConnectTimeReset(
    boost::bind(&Minho_Robot::onReset, this));
    _timeres_connection_ = event::Events::ConnectWorldReset(
    boost::bind(&Minho_Robot::onReset, this));
    getGameBallModel();
    
    // Print models in world
    std::vector<physics::ModelPtr> models_ = _model_->GetWorld()->GetModels();
    std::stringstream models_list;
    models_list << "Current Models in World:";
    for(unsigned int m = 0;m<models_.size();m++) models_list << "\n\t\t\t\tModel " << m << " -> " << models_[m]->GetName();
    models_list << "\n";
    ROS_INFO("%s",models_list.str().c_str());
}

void Minho_Robot::applyVelocities(math::Vector3 command)
{
    // Apply angular velocity
    if(command.z>=-100.0 && command.z<=100.0) angular_velocity_.z = mapVelocity(command.z,MAX_ANG_VEL); 
    else angular_velocity_.z = 0;
    
    // Apply linear velocity
    if((command.x>=0.0 && command.x<=100.0) && (command.y>=0.0 && command.y<=360.0)) {
        double angle = command.y*DEG_TO_RAD+M_PI/2.0+ model_pose_.rot.GetAsEuler().z;
        linear_velocity_.x = mapVelocity(command.x*cos(angle),MAX_LIN_VEL); 
        linear_velocity_.y = mapVelocity(command.x*sin(angle),MAX_LIN_VEL);        
    } else { linear_velocity_.x = linear_velocity_.y = 0.0; }
    
    _model_->SetLinearVel(linear_velocity_);
    _model_->SetAngularVel(angular_velocity_);
}

double Minho_Robot::mapVelocity(double percentage, double limit)
{
    return (percentage*limit)/100.0;
}


void Minho_Robot::autoRenameRobot()
{
    team_id_ = 1;
    physics::WorldPtr world = _model_->GetWorld(); 
    std::vector<physics::ModelPtr> models_list = world->GetModels();
    if(_model_->GetName().compare("minho_robot")==0){
        std::vector<int> used_team_ids; used_team_ids.clear();
        for(unsigned int id = 0; id < models_list.size(); id++){ // Generate used id's list
            // tag starts as : minho_robot and suffix is $(number) -> minho_robot$1
            std::string mod = models_list[id]->GetName().substr(0,11);
            if(mod.compare("minho_robot")==0 && models_list[id]->GetName().compare(_model_->GetName())!=0){
                used_team_ids.push_back(std::stoi(models_list[id]->GetName().substr(12,1),nullptr));
            }
        }
        std::stringstream new_name;
        if(used_team_ids.size()==0){ // first model to be loaded
            team_id_ = 1;
        } else { // find available ids between 1 and 6
            team_id_ = 1;
            std::sort (used_team_ids.begin(), used_team_ids.end()); //order used id vector
            for(unsigned int id = 0; id < used_team_ids.size(); id++){
                if(team_id_>=used_team_ids[id]) team_id_++;
                else {
                    break;
                }
            }
        }
        new_name << "minho_robot_" << std::to_string(team_id_);
        _model_->SetName(new_name.str());
    } else {
        std::string tid = _model_->GetName(); 
        char id = tid[tid.length()-1];
        team_id_ = id-'0';
    }
  
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
        std::stringstream new_uri; new_uri << "model://minhoteam_msl_robot/meshes/nr_" << std::to_string(team_id_)<< ".stl";
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
        std::stringstream new_uri; new_uri << "model://minhoteam_msl_robot/meshes/nr_" << std::to_string(team_id_)<< ".stl";
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
        std::stringstream new_uri; new_uri << "model://minhoteam_msl_robot/meshes/nr_" << std::to_string(team_id_)<< ".stl";
        number_uri->GetValue()->SetFromString(new_uri.str());
    }
    
    // Update model's SDF
    _model_->Update();
}


void Minho_Robot::onUpdate()
{
    static int kick_timer = 0;
    // lock resources
    control_info_mutex_.lock();
    model_pose_ = _model_->GetWorldPose(); // get robot position
    getGameBallModel(); // finds and computes distance to game ball

    // Apply defined velocities to the robot
    _model_->SetLinearVel(linear_velocity_);
    _model_->SetAngularVel(angular_velocity_);
    
    // Activate dribbling algorithm, if the ball is in possession and
    // dribling is activated
    if(kick_requested_ && has_game_ball_) kick_timer = 10;
    else if(kick_timer>0) kick_timer--;
    if(dribblers_on_ && has_game_ball_ && (kick_timer<=0)) dribbleGameBall();
    if(kick_requested_ && has_game_ball_) kickGameBall(kick_is_pass_,kick_force_,kick_dir_);
        
    // Publish robotInfo over ROS
    publishRobotInfo();
    // unlock resources
    control_info_mutex_.unlock();
}

void Minho_Robot::controlInfoCallback(const controlInfo::ConstPtr& msg)
{
    // lock resources
    control_info_mutex_.lock();
    
    if(teleop_active_){
        if(msg->is_teleop) {
            //Apply robot velocities
            linear_vel_ = msg->linear_velocity;
            mov_direction_ = msg->movement_direction;
            angular_vel_ = msg->angular_velocity;
            applyVelocities(math::Vector3(msg->linear_velocity, msg->movement_direction, msg->angular_velocity));
            //Apply dribbling
            dribblers_on_ = msg->dribbler_on;
            //Apply ball kicking
            if(msg->kick_strength>0){
                kick_requested_ = true;
                kick_force_ = msg->kick_strength;
                kick_dir_ = msg->kick_direction;
                kick_is_pass_ = msg->kick_is_pass;
                dribblers_on_ = false;
            }
        }
    } else { // Autonomous
        if(!msg->is_teleop) {
            //Apply robot velocities
            linear_vel_ = msg->linear_velocity;
            mov_direction_ = msg->movement_direction;
            angular_vel_ = msg->angular_velocity;
            applyVelocities(math::Vector3(msg->linear_velocity, msg->movement_direction, msg->angular_velocity));
            //Apply dribbling
            dribblers_on_ = msg->dribbler_on;
            //Apply ball kicking
            if(msg->kick_strength>0){
                kick_requested_ = true;
                kick_force_ = msg->kick_strength;
                kick_dir_ = msg->kick_direction;
                kick_is_pass_ = msg->kick_is_pass;
                dribblers_on_ = false;
            }
        }
    }
    
    // unlock resources
    control_info_mutex_.unlock();
}

void Minho_Robot::teleopCallback(const teleop::ConstPtr& msg)
{
    // lock resources
    tele_op_mutex_.lock();
    
    teleop_active_ = msg->set_teleop;
    if(teleop_active_)ROS_INFO("Teleop activated for '%s'.",_model_->GetName().c_str());
    else ROS_INFO("Teleop deactivated for '%s'.",_model_->GetName().c_str());
    

    // unlock resources
    tele_op_mutex_.unlock();
}


void Minho_Robot::message_queue_thread()
{
  static const double timeout = 0.01;
  while (_node_ros_->ok())
  {
    // Invoke all callbacks currently in the queue. If a callback was not ready to be called,
    // pushes it back onto the queue. This version includes a timeout which lets you specify
    // the amount of time to wait for a callback to be available before returning.
    message_queue_.callAvailable(ros::WallDuration(timeout));
  }
}

void Minho_Robot::getGameBallModel()
{
    physics::WorldPtr world = _model_->GetWorld(); 
    _ball_ = world->GetModel(BALL_MODEL_NAME);
    if(!_ball_) { has_game_ball_ = false; game_ball_in_world_ = false;}
    else { 
        ball_pose_ = _ball_->GetWorldPose(); 
        detectBallPossession(); 
        game_ball_in_world_ = true;
    }
}

void Minho_Robot::detectBallPossession()
{
    has_game_ball_ = false;
    ignition::math::Vector2<float> ball_position = ignition::math::Vector2<float>((float)ball_pose_.pos.x,(float)ball_pose_.pos.y);
   
    if(ball_pose_.pos.z>=-0.2 && ball_pose_.pos.z<= 0.22){ // If the ball is on game floor
        distance_to_ball_ = ball_position.Distance(ignition::math::Vector2<float>((float)model_pose_.pos.x,(float)model_pose_.pos.y));
        double direction = (std::atan2(ball_pose_.pos.y-model_pose_.pos.y,ball_pose_.pos.x-model_pose_.pos.x)-model_pose_.rot.GetAsEuler().z
                                     +(float)M_PI/2.0)*RAD_TO_DEG;
        while(direction>360.0) direction -= 360.0;
        while(direction<0) direction += 360.0;
        
        // Has to be in reach of the imaginary dribbles
        if((distance_to_ball_<= poss_threshold_distance_) && (direction>=150&&direction<=210)) has_game_ball_ = true;
        else has_game_ball_ = false;
    }
}

void Minho_Robot::publishRobotInfo()
{
    minho_team_ros::robotInfo msg;
    // Robot pose
    msg.robot_pose.x = model_pose_.pos.x;
    msg.robot_pose.y = model_pose_.pos.y;
    msg.robot_pose.z = model_pose_.rot.GetAsEuler().z + (float)M_PI/2.0;
    // Ball position
    if(game_ball_in_world_ && distance_to_ball_<=VISION_RANGE_RADIUS) { 
        // Add noise
        double min = 0.0, max = 0.5;
        double ratio = distance_to_ball_/VISION_RANGE_RADIUS;
        if(ratio <= 0.3) {min = 0.0; max = 0.03;}
        else if(ratio <= 0.6) {min = 0.0; max = 0.05;}
        else if(ratio <= 0.8) {min = 0.05; max = 0.2;}
        else if(ratio <= 1.0) {min = 0.2; max = 0.5;}
        //
        msg.ball_position.x = ball_pose_.pos.x+generateNoise(0.0,0.25,min,max);
        msg.ball_position.y = ball_pose_.pos.y+generateNoise(0.0,0.25,min,max);
        msg.sees_ball = true;
    }                
    // Ball sensor
    if(has_game_ball_) msg.has_ball = 1; else msg.has_ball = 0;
    // Imu
    while(msg.robot_pose.z>(2.0*M_PI)) msg.robot_pose.z -= (2.0*M_PI);
    while(msg.robot_pose.z<0) msg.robot_pose.z += (2.0*M_PI);
    msg.imu_value = msg.robot_pose.z*RAD_TO_DEG;
    // Obstacles
    msg.obstacles = detectObstacles();
    
    if(robot_info_pub_) robot_info_pub_.publish(msg);
}

void Minho_Robot::dribbleGameBall()
{
    double grip_force = 1.0;
    double linear_grip_reducer = 0.0;
    
    // linear movement
    if(mov_direction_>=0.0 && mov_direction_ <= 60.0) linear_grip_reducer = 0.0;
    else if(mov_direction_>=300.0 && mov_direction_ <= 360.0) linear_grip_reducer = 0.0;
    else {
        if(linear_vel_<40.0) linear_grip_reducer = 0.0;
        else {
            linear_grip_reducer = (1.0/50.0)*linear_vel_-(4.0/5.0);   
        }    
    }
    
    grip_force -= linear_grip_reducer;
    
    // max grip 2/3 of ball inside robot, minimum grip 1/3 of ball inside robot
    math::Vector3 robot_position = math::Vector3((float)model_pose_.pos.x,(float)model_pose_.pos.y,0.0);
    double robot_heading = model_pose_.rot.GetAsEuler().z+(float)M_PI/2.0;
    double distance_null = 0.21326;
    double distance = distance_null+((-0.14674*grip_force)+0.14674);
    math::Vector3 ball_position = robot_position + math::Vector3(distance*cos(robot_heading),distance*sin(robot_heading),0.11);
      
    _ball_->SetWorldPose(math::Pose(ball_position,math::Quaternion(0,0,0,0)));
    if(mov_direction_>90 && mov_direction_<270){
        double release_velocity = 0.5;
        _ball_->SetLinearVel(math::Vector3(-release_velocity*cos(robot_heading),-release_velocity*sin(robot_heading),0.0));
    }
    else _ball_->SetLinearVel(math::Vector3(0,0,0));
    // otherwise let the ball suffer forces 
}

void Minho_Robot::kickGameBall(bool pass, int strength, int direction)
{
    math::Vector3 kicking_vector;
    if(pass){
        double magnitude = mapVelocity(strength,MAX_BALL_VEL);
        double direction = mapVelocity(direction,20.0);
        double robot_heading = model_pose_.rot.GetAsEuler().z+(float)M_PI/2.0;
        kicking_vector.x = magnitude*cos(-direction+robot_heading);
        kicking_vector.y = magnitude*sin(-direction+robot_heading);
        kicking_vector.z = 0.0;
    } else {
        double shooting_angle = SHOOT_ANGLE;
        double magnitude = mapVelocity(strength,MAX_BALL_VEL);
        double magnitude_front = magnitude*cos(shooting_angle*DEG_TO_RAD);
        double direction = mapVelocity(direction,20.0);
        double robot_heading = model_pose_.rot.GetAsEuler().z+(float)M_PI/2.0;
        kicking_vector.x = magnitude_front*cos(-direction+robot_heading);
        kicking_vector.y = magnitude_front*sin(-direction+robot_heading);
        kicking_vector.z = magnitude*sin(shooting_angle*DEG_TO_RAD);
           
    }
    _ball_->SetLinearVel(kicking_vector);
    kick_requested_ = false;        
}

void Minho_Robot::initializePluginParameters(sdf::ElementPtr _sdf)
{
    if(_sdf->HasElement("max_linear_velocity")){
       _sdf->GetElement("max_linear_velocity")->GetValue()->Get(MAX_LIN_VEL);
    } else ROS_WARN("No maximum linear velocity parameter defined in plugin's SDF");
    
    if(_sdf->HasElement("max_angular_velocity")){
        _sdf->GetElement("max_angular_velocity")->GetValue()->Get(MAX_ANG_VEL);
    } else ROS_WARN("No maximum angular velocity parameter defined in plugin's SDF");
    
    if(_sdf->HasElement("max_ball_velocity")){
        _sdf->GetElement("max_ball_velocity")->GetValue()->Get(MAX_BALL_VEL);
    } else ROS_WARN("No maximum ball velocity parameter defined in plugin's SDF");
    
    if(_sdf->HasElement("ball_kick_angle")){
        _sdf->GetElement("ball_kick_angle")->GetValue()->Get(SHOOT_ANGLE);
    } else ROS_WARN("No ball kicking angle parameter defined in plugin's SDF");
    
    if(_sdf->HasElement("ball_model_name")){
        _sdf->GetElement("ball_model_name")->GetValue()->Get(BALL_MODEL_NAME);
    } else ROS_WARN("No ball model name parameter defined in plugin's SDF");
    
    if(_sdf->HasElement("vision_range_radius")){
        _sdf->GetElement("vision_range_radius")->GetValue()->Get(VISION_RANGE_RADIUS);
    } else ROS_WARN("No vision range radius parameter defined in plugin's SDF");
}

double Minho_Robot::generateNoise(double mean, double stdev, double min, double max)
{
    double noise = 0;
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator (seed);
    std::normal_distribution<double> distribution (mean,stdev);
    do{
        noise = distribution(generator);
    }while(noise<=min || noise>=max);
    return noise;
}

std::vector<minho_team_ros::position> Minho_Robot::detectObstacles()
{
    std::vector<minho_team_ros::position>obstacles;
    obstacles.clear();
    
    physics::WorldPtr world = _model_->GetWorld(); 
    std::vector<physics::ModelPtr> models_list = world->GetModels();
    minho_team_ros::position position;
    double min = 0.0, max = 0.5;
    double ratio = 0.0;
    
    for(unsigned int id = 0; id < models_list.size(); id++){ // Generate used id's list
        std::string mod = models_list[id]->GetName().substr(0,11);
        if((mod.compare("minho_robot")==0 || mod.compare("other_robot")==0) && models_list[id]->GetName().compare(_model_->GetName())!=0){
            math::Pose obstacle_pose = models_list[id]->GetWorldPose();
            math::Vector3 robot_position = math::Vector3((float)model_pose_.pos.x,(float)model_pose_.pos.y,0.0);
            double distance = robot_position.Distance(obstacle_pose.pos.x, obstacle_pose.pos.y, 0.0);
            ratio = distance/VISION_RANGE_RADIUS;
            if(ratio <= 0.3) {min = 0.0; max = 0.03;}
            else if(ratio <= 0.6) {min = 0.0; max = 0.05;}
            else if(ratio <= 0.8) {min = 0.05; max = 0.2;}
            else if(ratio <= 1.0) {min = 0.2; max = 0.5;}
            // Add gaussian noise
            position.x = obstacle_pose.pos.x+generateNoise(0.0,0.25,min,max); 
            position.y = obstacle_pose.pos.y+generateNoise(0.0,0.25,min,max);
            if(distance <= VISION_RANGE_RADIUS) obstacles.push_back(position);
        }
    }
    
    return obstacles;       
}

void Minho_Robot::onReset()
{
    _model_->SetWorldPose(initial_pose_);   
}
