#include "following_controller/lookfortarget.h"

namespace FOLLOWING
{
    LookforTargetServer::LookforTargetServer(ros::NodeHandle &nh, const std::string &action_name) : nh_(nh), as_(nh, action_name, boost::bind(&LookforTargetServer::ExecuteCB, this, _1), false), tfListener_(tfBuffer_), action_name_(action_name), is_active_(false)
    {
        cmdVelPub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel_x", 10);
        double rate = 10;
        double control_dt_ = 1.0 / rate;
        rot_pid_controller_ptr_ = std::make_unique<PID_controller>(1.0, 0.5, 0.2, 0.0, -0.2, 0.2, -0.2, 0.2, control_dt_);

        as_.start();
        ROS_INFO("Action server starts: %s", action_name.c_str());
    }

    LookforTargetServer::~LookforTargetServer()
    {
        if (is_active_)
        {
            ROS_INFO_STREAM("Stopping  action server: " << action_name_.c_str());
        }
    }

    LookforTargetServer::ExecuteCB(const lookfor_target_action::LookforTargetGoalConstPtr &goal)
    {
        ROS_INFO("Start lookfor_target_action");
        bool state = true; // true: anticlockwise, false: clockwise
        bool success = false;
        int progress = 0;
        is_active_ = true;
        ros::Rate rate(10);
        double initialYaw = 0.0;
        double theta = goal->angle / 2.0;

        geometry_msgs::TransformStamped transformStamped;
        try
        {
            transformStamped = tfBuffer_.lookupTransform("map", "base_link", ros::Time(0));
            initialYaw = tf2::getYaw(transformStamped.transform.rotation);
        }
        catch (const std::exception &ex)
        {
            ROS_ERROR("Failed to lookup transform: %s", ex.what());
            result_.success = false;
            result_.message = "Failed to lookup transform";
            as_.setAborted(result_);
            is_active_ = false;
            return;
        }

        while (ros::ok() && is_active_ && progress < 3)
        {
            transformStamped = tfBuffer_.lookupTransform("map", "base_link", ros::Time(0));

            if (as_.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO_STREAM("Action preempted: " << action_name_.c_str());
                result_.success = false;
                result_.message = "Action preempted";
                as_.setPreempted(result_);
                is_active_ = false;
                return;
            }

            if (state)
            {
                double currentYaw = tf2::getYaw(transformStamped.transform.rotation);
                // PublishFeedback(currentYaw);
                double rot = utils::rad2deg(currentYaw - initialYaw);
                double therr = theta - rot;
                if (!utils::IsDoubleEqualtoZero(therr, 0.1))
                {
                    double w = th_pid_controller_ptr_->calc_output(-therr, control_dt_) * 2.5;
                    double velYaw = w / 2.0;
                    geometry_msgs::Twist vel;
                    vel.angular.z = velYaw;
                    cmdVelPub_.publish(vel);
                }
                else
                {
                    geometry_msgs::Twist vel;
                    vel.angular.z = 0.0;
                    cmdVelPub_.publish(vel);
                    state = false;
                    initialYaw = currentYaw;
                    progress++;
                }
            }
            else
            {
                double currentYaw = tf2::getYaw(transformStamped.transform.rotation);
                // PublishFeedback(currentYaw);
                double rot = utils::rad2deg(currentYaw - initialYaw);
                double therr = rot - goal->angle;
                if (!utils::IsDoubleEqualtoZero(therr, 0.1))
                {
                    double w = th_pid_controller_ptr_->calc_output(-therr, control_dt_) * 2.5;
                    double velYaw = w / 2.0;
                    geometry_msgs::Twist vel;
                    vel.angular.z = velYaw;
                    cmdVelPub_.publish(vel);
                }
                else
                {
                    geometry_msgs::Twist vel;
                    vel.angular.z = 0.0;
                    cmdVelPub_.publish(vel);
                    state = true;
                    initialYaw = currentYaw;
                    progress++;
                }
            }

            if (progress == 3)
            {
                success = true;
                break;
            }

            rate.sleep();
        }

        if (success)
        {
            result_.success = true;
            result_.message = "lookfor_target_action completed";
            as_.setSucceeded(result_);
        }
        else
        {
            result_.success = false;
            result_.message = "lookfor_target_action failed";
            as_.setAborted(result_);
        }

        is_active_ = false;
    }

    LookforTargetServer::PublishFeedback(double current_yaw)
    {
        feedback_.current_yaw = current_yaw;
        as_.publishFeedback(feedback_);
    }

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

    void LookforTargetClient::FeedbackCB(const lookfor_target_action::LookforTargetActionFeedbackConstPtr &feedback)

    {
        ROS_INFO("Current yaw: %f", feedback->current_yaw);
    }

    void LookforTargetClient::SendGoal(const lookfor_target_action::LookforTargetActionGoal &goal)

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
        return ac_.getState().isActive();
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