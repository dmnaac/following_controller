#include "following_controller/lookfortargetclient.h"

namespace FOLLOWING
{
    LookforTargetClient::LookforTargetClient(ros::NodeHandle &nh, const std::string &action_name) : nh_(nh), ac_(nh, action_name, true), is_looking_for_target_(false)
    {
        ROS_INFO("Waiting for lookfor_target_action server");
        bool is_server_available = ac_.waitForServer(ros::Duration(5.0));
        if (is_server_available)
        {
            ROS_INFO("Connected to lookfor_target_action server");
        }
        else
        {
            ROS_ERROR("Failed to connect to lookfor_target_action server");
        }
    }

    LookforTargetClient::~LookforTargetClient()
    {
    }

    void LookforTargetClient::DoneCB(const actionlib::SimpleClientGoalState &state, const lookfor_target_action::LookforTargetResultConstPtr &result)
    {
        if (result->success)
        {
            ROS_WARN("Action completed but target may not be found");
        }
        else
        {
            ROS_WARN("Action failed: %s", result->message.c_str());
        }

        is_looking_for_target_ = false;
    }

    void LookforTargetClient::ActiveCB()
    {
        ROS_INFO("Action started");
    }

    void LookforTargetClient::FeedbackCB(const lookfor_target_action::LookforTargetFeedbackConstPtr &feedback)
    {
        ROS_INFO("Current yaw: %f", feedback->current_yaw);
    }

    void LookforTargetClient::SendGoal(const lookfor_target_action::LookforTargetGoal &goal)
    {
        ac_.sendGoal(goal,
                     boost::bind(&LookforTargetClient::DoneCB, this, _1, _2),
                     boost::bind(&LookforTargetClient::ActiveCB, this),
                     boost::bind(&LookforTargetClient::FeedbackCB, this, _1));

        is_looking_for_target_ = true;
    }

    void LookforTargetClient::CancelGoal()
    {
        if (IsActive())
        {
            ac_.cancelGoal();
            is_looking_for_target_ = false;
        }
        else
        {
            ROS_INFO("Action not active");
        }
    }

    bool LookforTargetClient::IsActive() const
    {
        std::string state_string = ac_.getState().toString();
        return state_string == "ACTIVE";
    }

    bool LookforTargetClient::IsServerConnected() const
    {
        return ac_.isServerConnected();
    }

    bool LookforTargetClient::GetState() const
    {
        return is_looking_for_target_;
    }
}