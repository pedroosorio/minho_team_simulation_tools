/*
 * packetStructureRefboxLogger.hpp
 *
 *  Created on: Nov 24, 2015
 *      Author: Tim Kouters
 */

#ifndef PACKETSTRUCTUREREFBOXLOGGER_HPP_
#define PACKETSTRUCTUREREFBOXLOGGER_HPP_

#include <cstdio>
#include <stdint.h>
#include <vector>

//#include "position2d.hpp"
//#include "vector2d.hpp"
//#include "vector3d.hpp"
#include "minho_team_ros/position.h"
#include "minho_team_ros/pose.h"
#include "minho_team_ros/velocity.h"

using minho_team_ros::pose;
using minho_team_ros::position;
using minho_team_ros::velocity;

typedef struct
{
    uint8_t     robotId;
    minho_team_ros::pose  pose;
    minho_team_ros::velocity  velocity;
    minho_team_ros::pose  targetPose;
    std::string intention;
    float       batteryLevel;
    bool        hasBall;
} robotStructure;
typedef std::vector<robotStructure> robotList;

typedef struct
{
    minho_team_ros::pose position;
    minho_team_ros::velocity velocity;
    float    confidence;
} ballStructure;
typedef std::vector<ballStructure> ballList;

typedef struct
{
    minho_team_ros::position position;
    minho_team_ros::velocity velocity;
    float    radius;
    float    confidence;
} obstacleStructure;
typedef std::vector<obstacleStructure> obstacleList;

typedef struct
{
    std::string type;
    std::string teamName;
    std::string globalIntention;
    robotList robots;
    ballList balls;
    obstacleList obstacles;
    size_t age;
} packetStructureDeserialized;

#endif /* PACKETSTRUCTUREREFBOXLOGGER_HPP_ */
