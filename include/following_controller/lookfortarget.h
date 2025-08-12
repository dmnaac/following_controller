#ifndef FOLLOWING_CONTROLLER_LOOKFORTARGET_H
#define FOLLOWING_CONTROLLER_LOOKFORTARGET_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>
#include "following_controller/LookforTargetAction.h"

class LookforTargetServer
{
private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<following_controller::LookforTargetAction> as_;
    ros::Publisher cmdVelPub_;

    std::string action_name_;
    following_controller::LookforTargetActionFeedback feedback_;
    following_controller::LookforTargetActionResult result_;

    float current_angle_;
    bool going_on_;

    void ExecuteCB(const following_controller::LookforTargetActionGoalConstPtr &goal);

public:
    LookforTargetServer(/* args */);
    ~LookforTargetServer();

    SetBreak(bool goingOn);
};

class LookforTargetClient
{
private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionClient<following_controller::LookforTargetAction> ac_;

    void DoneCB(const actionlib::SimpleClientGoalState &state, const following_controller::LookforTargetActionResultConstPtr &result);
    void ActiveCB();
    void FeedbackCB(const following_controller::LookforTargetActionFeedbackConstPtr &feedback);

public:
    LookforTargetClient(/* args */);
    ~LookforTargetClient();
};

#endif // FOLLOWING_CONTROLLER_LOOKFORTARGET_H