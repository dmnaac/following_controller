#ifndef FOLLOWING_CONTROLLER_LOOKFORTARGETCLIENT_H
#define FOLLOWING_CONTROLLER_LOOKFORTARGETCLIENT_H

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
    class LookforTargetClient
    {
    private:
        ros::NodeHandle nh_;
        actionlib::SimpleActionClient<lookfor_target_action::LookforTargetAction> ac_;

        bool is_looking_for_target_;

        void DoneCB(const actionlib::SimpleClientGoalState &state, const lookfor_target_action::LookforTargetResultConstPtr &result);
        void ActiveCB();
        void FeedbackCB(const lookfor_target_action::LookforTargetFeedbackConstPtr &feedback);

    public:
        LookforTargetClient(ros::NodeHandle &nh, const std::string &action_name);

        ~LookforTargetClient();

        void SendGoal(const lookfor_target_action::LookforTargetGoal &goal);
        void CancelGoal();
        bool IsActive() const;
        bool IsServerConnected() const;
        bool GetState() const;
    };
}

#endif // FOLLOWING_CONTROLLER_LOOKFORTARGETCLIENT_H