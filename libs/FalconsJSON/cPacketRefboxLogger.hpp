/*
 * cPacketRefboxLogger.hpp
 *
 *  Created on: Nov 24, 2015
 *      Author: Tim Kouters
 */

#ifndef CPACKETREFBOXLOGGER_HPP_
#define CPACKETREFBOXLOGGER_HPP_

#include <stddef.h>
#include <string>
#include <json-c/json.h>

//#include "position2d.hpp"
//#include "vector2d.hpp"
//#include "vector3d.hpp"
#include "minho_team_ros/pose.h"
#include "minho_team_ros/position.h"
#include "minho_team_ros/interAgentInfo.h"
#include "packetStructureRefboxLogger.hpp"

using minho_team_ros::pose;
using minho_team_ros::position;
using minho_team_ros::interAgentInfo;

class cPacketRefboxLogger
{
    public:
        cPacketRefboxLogger(std::__cxx11::string team_name);
        ~cPacketRefboxLogger();

        size_t getSize();

        void getSerialized(std::string &packet);

        /*
         * Team level setters
         */
        void setType(const std::string type);
        void setTeamIntention(const std::string intention);

        /*
         * Robot level setters
         */
        void setRobotPose(const uint8_t robotId, const minho_team_ros::pose pose);
        void setRobotTargetPose(const uint8_t robotId, const minho_team_ros::pose targetPose);
        void setRobotVelocity(const uint8_t robotId, const minho_team_ros::velocity velocity);
        void setRobotIntention(const uint8_t robotId, const std::string intention);
        void setRobotBatteryLevel(const uint8_t robotId, const float level);
        void setRobotBallPossession(const uint8_t robotId, const bool hasBall);

        /*
         * Ball setters
         */
        void addBall(const minho_team_ros::pose position, const minho_team_ros::velocity velocity, const float confidence);

        /*
         * Obstacle setters
         */
        void addObstacle(const minho_team_ros::position position, const minho_team_ros::velocity velocity, const float confidence);

        /*
         * Global setters
         */
        void setAgeMilliseconds(const size_t age);

        void cleanBallsAndObstacles();
        void updateRobotInformation(minho_team_ros::interAgentInfo info);
        void removeRobot(int robotId);
    private:
        packetStructureDeserialized _mPacket;
        json_object *_jsonObject;

        /* Robot functions */
        void isRobotPresent(const uint8_t robotId, bool &isPresent, size_t &index);
        void addRobot(const uint8_t);

        /* JSON functions */
        void cleanupJSONObject();
        void generateJSON();
        void addRobotsJSON(json_object *obj);
        void addBallsJSON(json_object *obj);
        void addObstaclesJSON(json_object *obj);
};


#endif /* CPACKETREFBOXLOGGER_HPP_ */
