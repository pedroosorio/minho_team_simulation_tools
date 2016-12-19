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
GZ_REGISTER_MODEL_PLUGIN(Minho_Robot);

/// \brief Constructor. Initialized deafult variables for various variables
Minho_Robot::Minho_Robot()
{
    initial_pose_ = math::Pose(0.0,-6.4*(-Y_AXIS_MULTIPLIER),0.0,0.0,0.0,0.0);
    linear_velocity_ = angular_velocity_ = math::Vector3(0.0,0.0,0.0);
    team_id_ = 0; //null
    is_ros_initialized_ = false;
    teleop_active_ = false;
    game_ball_in_world_ = false;
    poss_threshold_distance_ = 0.3;
    kick_requested_ = false;
    
    // Default values
    MAX_LIN_VEL = 2.5;
    MAX_ANG_VEL = 15.0;
    MAX_BALL_VEL = 8.0;
    SHOOT_ANGLE = 0.0;
    BALL_MODEL_NAME = "RoboCup MSL Ball";
    VISION_RANGE_RADIUS = 5.0;
    MAX_BACKWARDS_VEL = GRIP_DECAY = MAX_ROTATION_VEL = 30.0;
}

/// \brief Destructor
Minho_Robot::~Minho_Robot()
{
    for(int i=0;i<childs.size();i++)childs[i].terminate();
    event::Events::DisconnectWorldUpdateBegin(_update_connection_);
    // Removes all callbacks from the queue. Does not wait for calls currently in progress to finish. 
    message_queue_.clear();
    // Disable the queue, meaning any calls to addCallback() will have no effect. 
    message_queue_.disable();
    message_callback_queue_thread_.join();
    _node_ros_->shutdown();
    delete _node_ros_;
}

/// \brief Plugin Load function. Initializes all ros topics for the robot model,
/// also starting message queue thread. Connects gazebo events like world update,
/// time reset and world reset
/// \param _parent - Model pointer to the model defining this plugin
/// \param _sdf - pointer to the SDF of the model
void Minho_Robot::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    // Rename model to specification
    _model_ = _parent;
    _node_ = transport::NodePtr(new transport::Node());
    _node_->Init(_model_->GetWorld()->GetName());
    model_id_ = _model_->GetId();
    // Stop simulation to prevent odd behaviors
    gazebo::physics::WorldPtr world = _model_->GetWorld();
    world_mutex_ = world->GetSetWorldPoseMutex();
    bool pauseState = world->IsPaused();
    world->SetPaused(true);
    world->DisableAllModels();
    world->EnablePhysicsEngine(false);

    initializePluginParameters(_sdf);
    autoRenameRobot();
    
    ROS_WARN("#############################################################");
    ROS_WARN("Loading Minho_Robot Plugin for '%s' ...", _model_->GetName().c_str());
    
    if (!ros::isInitialized()){
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin.");
        is_ros_initialized_ = false;
    } else is_ros_initialized_ = true;
    
    // Initialize ROS interface
    if(!is_ros_initialized_){
        ROS_ERROR("Plugin ROS failed to start on '%s'.", _model_->GetName().c_str());
    } else {
        ROS_INFO("Plugin ROS successfuly initialized.");
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
        
        std::stringstream kick_service_topic;
        kick_service_topic << "minho_gazebo_robot" << std::to_string(team_id_) << "/requestKick";
        kick_service = _node_ros_->advertiseService(kick_service_topic.str(),&Minho_Robot::kickServiceCallback,this);
        
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
    
    world->EnablePhysicsEngine(true);
    world->EnableAllModels();
    world->SetPaused(pauseState);
    
    //Place the robot in the initial position regarding its team_id_
    initial_pose_.pos.x = 0.8*(float)team_id_;
    _model_->SetWorldPose(initial_pose_);
    
    // Boots other ROS Nodes as coms, control, AI ...
    bootROSNodes(_sdf);
    
    //setupSensors();
}

/// \brief Applies the desired velocities to the robot, given
/// the linear velocity, direction of movement and angular velocity
/// \param command - has 3 components: (1) is linear velocity [0~100]
/// (2) is direction of movement [0~360] and (3) is angular velocity [-100~100]
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

/// \brief maps a velocity of the maximum velocity given a percentage.
/// \param percentage - percentage of the maximum velocity to be applied
/// \param limit - max velocity to be applied
/// \return mapped value/velocity given a limit and a percentage
double Minho_Robot::mapVelocity(double percentage, double limit)
{
    return (percentage*limit)/100.0;
}

/// \brief gets a list of models in the world and renames itself, accordingly to the
 /// existing robots already spawned.
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

/// \brief called by event signal every server simulation iteration. Used to reset 
/// parameters like velocities and others
void Minho_Robot::onUpdate()
{
    static int kick_timer = 0;
    // lock resources
    control_info_mutex_.lock();
    
    model_pose_ = _model_->GetWorldPose(); // get robot position
    model_pose_.pos.y *= Y_AXIS_MULTIPLIER;
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
    last_state = current_state;
    // unlock resources
    control_info_mutex_.unlock();
}

/// \brief called by event signal when a model or server reset happens
void Minho_Robot::onReset()
{
   _model_->SetWorldPose(initial_pose_);  
}

/// \brief callback to receive ROS messages published over the matching ros topic,
/// in order to retrieve data about comands for robot motion.
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
        }
    }
    
    // unlock resources
    control_info_mutex_.unlock();
}

/// \brief function to actuate kicker, in order to kick the ball. Only kicks if
/// the robot detects that has the ball inside
/// \param req - request data received in requestKick service
/// \param res - response data, flaggin if the kick was taken or not
bool Minho_Robot::kickServiceCallback(requestKick::Request &req,requestKick::Response &res)
{
   //Apply ball kicking
   if(req.kick_strength>0){
       kick_requested_ = true;
       kick_force_ = req.kick_strength;
       kick_dir_ = req.kick_direction;
       kick_is_pass_ = req.kick_is_pass;
       dribblers_on_ = false;
   }
   
   res.kicked = has_game_ball_;
   return true;
}
/// \brief callback to receive ROS messages published over the matching ros topic,
/// in order to retrieve data about comands for robot motion.
void Minho_Robot::teleopCallback(const teleop::ConstPtr& msg)
{
    // lock resources
    tele_op_mutex_.lock();
    
    teleop_active_ = msg->set_teleop;
    if(teleop_active_)ROS_INFO("Teleop activated for '%s'.",_model_->GetName().c_str());
    else { 
      ROS_INFO("Teleop deactivated for '%s'.",_model_->GetName().c_str());
      linear_velocity_ = angular_velocity_ = math::Vector3(0.0,0.0,0.0);
      kick_requested_ = false;
    }
    
    // unlock resources
    tele_op_mutex_.unlock();
}

/// \brief thread to queue incoming data to callback queue, that subsequently calls 
/// the respective callback, passing the matching data to it.
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

/// \brief searches for game ball inside the world and stores a pointer to the model
void Minho_Robot::getGameBallModel()
{
    physics::WorldPtr world = _model_->GetWorld(); 
    _ball_ = world->GetModel(BALL_MODEL_NAME);
    if(!_ball_) { has_game_ball_ = false; game_ball_in_world_ = false;}
    else { 
        ball_pose_ = _ball_->GetWorldPose(); 
        // Compute center position
        
        ball_pose_.pos.y *= Y_AXIS_MULTIPLIER;
        detectBallPossession(); 
        game_ball_in_world_ = true;
    }
}

/// \brief runs ball detection "sensor", using ball and robot pose's, computing the
/// distance between them, comparing with a threshold.
void Minho_Robot::detectBallPossession()
{
    has_game_ball_ = false;
    ignition::math::Vector2<float> ball_position = ignition::math::Vector2<float>((float)ball_pose_.pos.x,(float)ball_pose_.pos.y);
   
    if(ball_pose_.pos.z>=-0.2 && ball_pose_.pos.z<= 0.22){ // If the ball is on game floor
        distance_to_ball_ = ball_position.Distance(ignition::math::Vector2<float>((float)model_pose_.pos.x,(float)model_pose_.pos.y));
        
        double robot_orientation = -model_pose_.rot.GetAsEuler().z+M_PI;
        while(robot_orientation>(2.0*M_PI)) robot_orientation -= (2.0*M_PI);
        while(robot_orientation<0) robot_orientation += (2.0*M_PI);
        robot_orientation *= (180.0/M_PI);
        
        double direction = std::atan2(ball_pose_.pos.y-model_pose_.pos.y,ball_pose_.pos.x-model_pose_.pos.x)*(180.0/M_PI)-robot_orientation;
        while(direction<0) direction += 360.0;
        while(direction>360.0) direction -= 360.0;
        
        // Has to be in reach of the imaginary grabbers
        if((distance_to_ball_<= poss_threshold_distance_) && (direction>=60&&direction<=120)) has_game_ball_ = true;
        else has_game_ball_ = false;
    }
}

/// \brief creates a ROS message, updates all the information and sends it through the
/// publisher
void Minho_Robot::publishRobotInfo()
{  
    // Robot pose
    current_state.robot_pose.x = model_pose_.pos.x;
    current_state.robot_pose.y = model_pose_.pos.y;
    current_state.robot_pose.z = -model_pose_.rot.GetAsEuler().z+M_PI;
    // Orientation transposed to values used in our referential
    // Ball position
    if(game_ball_in_world_ && distance_to_ball_<=VISION_RANGE_RADIUS) { 
        // Add noise
        double min = 0.0, max = 0.5;
        double ratio = distance_to_ball_/VISION_RANGE_RADIUS;
        if(ratio <= 0.3) {min = 0.0; max = 0.03;}
        else if(ratio <= 0.6) {min = 0.0; max = 0.05;}
        else if(ratio <= 0.8) {min = 0.05; max = 0.2;}
        else if(ratio <= 1.0) {min = 0.2; max = 0.35;}
       
        double error = generateNoise(0.0,0.25,min,max);
        double direction = std::atan2(ball_pose_.pos.y-model_pose_.pos.y
        ,ball_pose_.pos.x-model_pose_.pos.x);
               
        current_state.ball_position.x = ball_pose_.pos.x+error*cos(direction);
        current_state.ball_position.y = ball_pose_.pos.y+error*sin(direction);
        
        current_state.sees_ball = true;
    } else current_state.sees_ball = false;             
    // Ball sensor
    if(has_game_ball_) current_state.has_ball = 1; else current_state.has_ball = 0;
    // Imu
    while(current_state.robot_pose.z>(2.0*M_PI)) current_state.robot_pose.z -= (2.0*M_PI);
    
    while(current_state.robot_pose.z<0) current_state.robot_pose.z += (2.0*M_PI);
    
    current_state.robot_pose.z = current_state.robot_pose.z*(180.0/M_PI);
    // Obstacles
    current_state.obstacles = detectObstacles();
    //mockObstacleDetection();  
    computeVelocities();
    
    if(robot_info_pub_) robot_info_pub_.publish(current_state);
}

/// \brief dribbles the ball, given the velocity vector of the robot
void Minho_Robot::dribbleGameBall()
{
    double grip_force = 1.0;
    double linear_grip_reducer = 0.0;
    
    // linear movement
    if(mov_direction_>=0.0 && mov_direction_ <= 60.0) linear_grip_reducer = 0.0;
    else if(mov_direction_>=300.0 && mov_direction_ <= 360.0) linear_grip_reducer = 0.0;
    else {
        if(linear_vel_<MAX_BACKWARDS_VEL) linear_grip_reducer = 0.0;
        else {
            linear_grip_reducer = (1.0/GRIP_DECAY)*linear_vel_-(MAX_BACKWARDS_VEL/GRIP_DECAY);   
        }    
    }
    
    if(linear_grip_reducer>1.0) linear_grip_reducer = 1.0;
    
    double rotation_grip_reducer = 0.0;
    // linear movement
     if(abs(angular_vel_)<MAX_ROTATION_VEL) rotation_grip_reducer = 0.0;
     else {
         rotation_grip_reducer = (1.0/GRIP_DECAY)*abs(angular_vel_)-(MAX_ROTATION_VEL/GRIP_DECAY);   
     }    
    
    if(linear_grip_reducer>1.0) linear_grip_reducer = 1.0;
    if(rotation_grip_reducer>1.0) rotation_grip_reducer = 1.0;
    grip_force -= (linear_grip_reducer+rotation_grip_reducer);
    
    // max grip 2/3 of ball inside robot, minimum grip 1/3 of ball inside robot
    math::Vector3 robot_position = math::Vector3((float)model_pose_.pos.x,(float)model_pose_.pos.y*Y_AXIS_MULTIPLIER,0.0);
    double robot_heading = model_pose_.rot.GetAsEuler().z+(float)M_PI/2.0;
    double distance_null = 0.21326;
    double distance = distance_null+((-0.14674*grip_force)+0.14674);
    math::Vector3 ball_position = robot_position + math::Vector3(distance*cos(robot_heading),distance*sin(robot_heading),0.11);
      
    _ball_->SetWorldPose(math::Pose(ball_position,math::Quaternion(0,0,0,0)));
    if(mov_direction_>60 && mov_direction_<300){
        double release_velocity = 0.05*mapVelocity(linear_vel_,MAX_ANG_VEL);
        _ball_->SetLinearVel(math::Vector3(-release_velocity*cos(robot_heading),-release_velocity*sin(robot_heading),0.0));
    }
    else _ball_->SetLinearVel(math::Vector3(0,0,0));
    // otherwise let the ball suffer forces 
}

/// \brief kicks the ball, through the floor (passing) or the air (shoothing)
/// allowing a slight variation in the direction of kicking
/// \param pass - define wether the kick is a pass or not
/// \param strength - define the strength of the kick
/// \param direction - define the direction of the kick
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

/// \brief reads and parses the elements passed to the plugin inside the model's sdf
/// and initializes the matching variables
/// \param _sdf - pointer to sdf element containing
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
    
    if(_sdf->HasElement("max_backwards_grip_vel")){
        _sdf->GetElement("max_backwards_grip_vel")->GetValue()->Get(MAX_BACKWARDS_VEL);
    } else ROS_WARN("No max grip backwards velocity parameter defined in plugin's SDF");
    
    if(_sdf->HasElement("max_rotation_grip_vel")){
        _sdf->GetElement("max_rotation_grip_vel")->GetValue()->Get(MAX_ROTATION_VEL);
    } else ROS_WARN("No max grip rotation velocity parameter defined in plugin's SDF");
    
    if(_sdf->HasElement("grip_decay")){
        _sdf->GetElement("grip_decay")->GetValue()->Get(GRIP_DECAY);
    } else ROS_WARN("No vision range radius parameter defined in plugin's SDF");
}

/// \brief generates random noise to add gaussian noise to a variable read from the 
/// world like ball position and obstacles position
/// \param mean - mean of the error to be generated
/// \param stdev - standard deviation of the error to 
/// \param min - minimum value of the output value
/// \param max - maximum value of the output value
/// \return generated noise value
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

/// \brief detects obstacles in the view range, whether they being friendly or foe,
/// at this point, in reality, the robot doesn't distinguish between friends or foes
/// return vector of positions containing the position of the detected obstalces
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
            obstacle_pose.pos.y *= Y_AXIS_MULTIPLIER;
            math::Vector3 robot_position = math::Vector3((float)model_pose_.pos.x,(float)model_pose_.pos.y,0.0);
            double distance = robot_position.Distance(obstacle_pose.pos.x, obstacle_pose.pos.y, 0.0);
            
            if(distance <= VISION_RANGE_RADIUS){
               ratio = distance/VISION_RANGE_RADIUS;
               if(ratio <= 0.3) {min = 0.0; max = 0.03;}
               else if(ratio <= 0.6) {min = 0.0; max = 0.05;}
               else if(ratio <= 0.8) {min = 0.05; max = 0.2;}
               else if(ratio <= 1.0) {min = 0.2; max = 0.3;}
               // Add gaussian noise
               double error = generateNoise(0.0,0.25,min,max);
               double direction = std::atan2(obstacle_pose.pos.y-model_pose_.pos.y,obstacle_pose.pos.x-model_pose_.pos.x);
               
               position.x = obstacle_pose.pos.x+error*cos(direction);
               position.y = obstacle_pose.pos.y+error*sin(direction);
               obstacles.push_back(position);
            }
        }
    }
    
    return obstacles;       
}

/// \brief computes both ball and robot velocities to send in robotInfo using
/// a simple low pass filter approach to reduce noisy estimates
void Minho_Robot::computeVelocities()
{
   int it_limit = 4;
   static int iteration = it_limit;
   float time_interval = 0.03125;
   if(iteration<(it_limit-1)) iteration++;
   else {
      iteration = 0;
      float lpf_weight = 0.9;
      float lpf_minor = 1-lpf_weight;
      // First, compute the robot velocities based on final localization estimate
      // ########################################################################
      // Time interval between estimates is requiredTiming = 33ms/30Hz
      current_state.robot_velocity.x = lpf_weight*((current_state.robot_pose.x-last_vel_state.robot_pose.x)/(time_interval*(float)it_limit))+lpf_minor*last_vel_state.robot_velocity.x;  
      current_state.robot_velocity.y = lpf_weight*((current_state.robot_pose.y-last_vel_state.robot_pose.y)/(time_interval*(float)it_limit))+lpf_minor*last_vel_state.robot_velocity.y; 
      current_state.robot_velocity.w = lpf_weight*((current_state.robot_pose.z-last_vel_state.robot_pose.z)/(time_interval*(float)it_limit))+lpf_minor*last_vel_state.robot_velocity.z; 
      // ########################################################################
      if(current_state.robot_velocity.x>2.5) current_state.robot_velocity.x = 0;
      if(current_state.robot_velocity.y>2.5) current_state.robot_velocity.y = 0;
      
      lpf_weight = 0.8;
      if(distance_to_ball_>(2.0*VISION_RANGE_RADIUS)/3.0) lpf_weight = 0.4;
      lpf_minor = 1-lpf_weight;
      current_state.ball_velocity.x = lpf_weight*((current_state.ball_position.x-last_vel_state.ball_position.x)/(time_interval*(float)it_limit))+lpf_minor*last_vel_state.ball_velocity.x;  
      current_state.ball_velocity.y = lpf_weight*((current_state.ball_position.y-last_vel_state.ball_position.y)/(time_interval*(float)it_limit))+lpf_minor*last_vel_state.ball_velocity.y; 
      
      
      last_vel_state = current_state;
   }
}

/// \brief boots indicated ROS nodes to add functionalities to the model
/// This nodes have to be specified in plugin's SDF correctly, naming the
/// package, node names, flags and placed in boot order
/// \param _sdf - sdf struct containing plugin's data, containing Robot parameters
/// and ROS boot node info
void Minho_Robot::bootROSNodes(sdf::ElementPtr _sdf)
{
   ROS_WARN("Booting complementary ROS Nodes ...");
   childs.clear();
   std::string executable_name = "rosrun";
   sdf::ElementPtr boot_list;
   if(_sdf->HasElement("ros_boot")){
     boot_list = _sdf->GetElement("ros_boot");
   } else { ROS_WARN("No ROS Node Boot List found."); return; }
   
   sdf::ElementPtr node;
   if(boot_list->HasElement("node")){
      node = boot_list->GetFirstElement();
      while(node){
         std::string package_name = "";
         std::string node_name = "";
         std::vector<std::string> flags_str; 
         // Parse information
         if(node->HasElement("package")){
            package_name = node->GetElement("package")->GetValue()->GetAsString();
            if(node->HasElement("name")){
               node_name = node->GetElement("name")->GetValue()->GetAsString();
               flags_str.clear();
               
               sdf::ElementPtr flags;
               if(node->HasElement("flags")){
                  flags = node->GetElement("flags");
                  sdf::ElementPtr flag;
                  flag = flags->GetFirstElement();
                  while(flag){
                     flags_str.push_back(assertFlagValue(flag->GetValue()->GetAsString()));
                     flag = flag->GetNextElement();
                  }
               } else ROS_WARN("No flags in Boot List for %s", node_name.c_str());
               
               //Add defined node to boot list
               //ROS_Boot_List
               std::vector<std::string> args;
               std::string exe = boost::process::find_executable_in_path(executable_name); 
               args.push_back(package_name); args.push_back(node_name);
               ROS_INFO("Booting %s %s",package_name.c_str(), node_name.c_str());
               ROS_INFO(" Flags:");
               for(int i=0;i<flags_str.size();i++) {
                  ROS_INFO("   %s",flags_str[i].c_str());
                  args.push_back(flags_str[i]);
               }
             
               childs.push_back(boost::process::create_child(exe,args)); 
            } else ROS_ERROR("Error in boot node definition, missing name.");
         } else ROS_ERROR("Error in boot node definition, missing package.");
         node = node->GetNextElement();
      }
   } else { ROS_WARN("No ROS Nodes found in Boot List."); return; }

   ROS_INFO("ROS Node Boot complete.");
   ROS_WARN("#############################################################");
}

/// \brief asserts flag value for ros node booting procedure. It can translate values
/// started with '$' with plugin run time variables
/// \param value - initial parameter value to be asserted
/// \return - string with the asserted parameter
std::string Minho_Robot::assertFlagValue(std::string value)
{
   if((value.size()>=1) && (value[0]!='$')) return value;
   
   if(!strcmp(value.c_str(),"$ID")) { return std::to_string(team_id_); }
   
   ROS_ERROR("Error in flag assert.");
   return "";
}

    
    
/// \brief setus up model sensors, performing detection and type identification
void Minho_Robot::setupSensors()
{
   // Check model sensors
   sensors_.clear();
   gazebo::sensors::SensorManager *manager = gazebo::sensors::SensorManager::Instance();
   if(manager){
   std::vector<gazebo::sensors::SensorPtr> sensors = manager->GetSensors();
      for(int i=0;i<sensors.size();i++){
         if(!_model_->GetName().compare(sensors[i]->ParentName().substr(0,sensors[i]->ParentName().find(":")))){
            sensors_.push_back(sensors[i]);
         }
      }
   }
   
   ROS_INFO("Found %lu sensors attached to %s",sensors_.size(),_model_->GetName().c_str());
   
   // Look for obstacle_detector sensor first
   for(int i=0;i<sensors_.size();i++){
      if(sensors_[i]->Name().compare("obstacle_detector")){
         obstacle_detector = dynamic_cast<gazebo::sensors::RaySensor *>(sensors_[i].get());
      }
   }
}    
   
/// \brief reads ray sensor to perform mock obstacle detection. It pushes the position
/// of obstacles into current_state(robotInfo)
void Minho_Robot::mockObstacleDetection()
{
   std::vector<double> ranges;
   ranges.clear();
   if(obstacle_detector) obstacle_detector->Ranges(ranges);
   
   position temp, robot_pos;
   robot_pos.x = current_state.robot_pose.x;
   robot_pos.y = current_state.robot_pose.y;
   float robot_heading = current_state.robot_pose.z*(M_PI/180.0);
   float angle = -M_PI;
   float step = obstacle_detector->AngleResolution();
   
   for(int i=0;i<ranges.size();i++){
      if(ranges[i]>0.30 && ranges[i]<VISION_RANGE_RADIUS){
         current_state.obstacles.push_back(mapPointToWorld(robot_pos,
                                                           robot_heading,
                                                           ranges[i],
                                                           angle));   
      }
      angle += step;
   }
}
  
/// \brief mapps a detected point relative to the robot to the world position
/// \param robot - position of the robot in the field, in meters
/// \param robot_heading - heading of the robot in º
/// \param dist - detected distance in meters
/// \param theta - angle of the detected point in relation to world's 0º
/// \return - position of the mapped point in meters
position Minho_Robot::mapPointToWorld(position robot, float robot_heading, float dist, float theta)
{
   double pointRelX = dist*cos(theta);
   double pointRelY = dist*sin(theta);
   position mappedPoint;
   mappedPoint.x = robot.x-cos(robot_heading)*pointRelX-sin(robot_heading)*pointRelY;
   mappedPoint.y = robot.y-sin(robot_heading)*pointRelX+cos(robot_heading)*pointRelY;
   return mappedPoint;
}
   
