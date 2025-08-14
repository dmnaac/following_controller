#ifndef FOLLOWING_CONTROLLER_LOOKFORTARGETSERVER_H
#define FOLLOWING_CONTROLLER_LOOKFORTARGETSERVER_H

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
#include "following_controller/pid_controller.h"
#include "following_controller/utils.h"

#include "lookfor_target_action/LookforTargetAction.h"

namespace FOLLOWING
{
    class LookforTargetServer
    {
    private:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<lookfor_target_action::LookforTargetAction> as_;
        ros::Publisher cmdVelPub_;
        tf2_ros::Buffer tfBuffer_;
        tf2_ros::TransformListener tfListener_;

        std::string action_name_;
        lookfor_target_action::LookforTargetFeedback feedback_;
        lookfor_target_action::LookforTargetResult result_;

        float current_angle_;
        bool is_active_;
        double control_dt_;

        std::unique_ptr<PID_controller> rot_pid_controller_ptr_;

        void ExecuteCB(const lookfor_target_action::LookforTargetGoalConstPtr &goal);
        void PublishFeedback(double current_yaw);

    public:
        LookforTargetServer(ros::NodeHandle &nh, const std::string &action_name);
        ~LookforTargetServer();
    };
}

#endif // FOLLOWING_CONTROLLER_LOOKFORTARGETSERVER_H