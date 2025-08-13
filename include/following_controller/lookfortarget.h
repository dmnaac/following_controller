#ifndef FOLLOWING_CONTROLLER_LOOKFORTARGET_H
#define FOLLOWING_CONTROLLER_LOOKFORTARGET_H

#include <ros/ros.h>
#include <cmath>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include "following_controller/LookforTargetAction.h"
#include "following_controller/pid_controller.h"
#include "following_controller/utils.h"

class LookforTargetServer
{
private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<following_controller::LookforTargetAction> as_;
    ros::Publisher cmdVelPub_;
    tf2_ros::Buffer tfBuffer_;
    tr2_ros::TransformListenner tfListener_;

    std::string action_name_;
    following_controller::LookforTargetActionFeedback feedback_;
    following_controller::LookforTargetActionResult result_;

    float current_angle_;
    bool is_active_;
    double control_dt_;

    std::unique_ptr<PID_controller> rot_pid_controller_ptr_;

    void ExecuteCB(const following_controller::LookforTargetActionGoalConstPtr &goal);
    void PublishFeedback(double current_yaw);

public:
    LookforTargetServer(ros::NodeHandle &nh, const std::string &action_name);
    ~LookforTargetServer();
};

class LookforTargetClient
{
private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionClient<following_controller::LookforTargetAction> ac_;
    bool is_looking_for_target_;

    void DoneCB(const actionlib::SimpleClientGoalState &state, const following_controller::LookforTargetActionResultConstPtr &result);
    void ActiveCB();
    void FeedbackCB(const following_controller::LookforTargetActionFeedbackConstPtr &feedback);

public:
    LookforTargetClient(/* args */);
    ~LookforTargetClient();

    void SendGoal(const following_controller::LookforTargetActionGoal &goal);
    void CancelGoal();
    bool IsActive() const;
    bool IsServerConnected() const;
    bool GetState() const;
};

#endif // FOLLOWING_CONTROLLER_LOOKFORTARGET_H