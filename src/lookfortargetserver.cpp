#include "following_controller/lookfortargetserver.h"

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

    void LookforTargetServer::ExecuteCB(const lookfor_target_action::LookforTargetGoalConstPtr &goal)
    {
        ROS_INFO("Start execution");
        bool state = true; // true: anticlockwise, false: clockwise
        bool success = false;
        int progress = 0;
        is_active_ = true;
        ros::Rate rate(10);
        double initialYaw = 0.0;

        double theta_rad = FOLLOWING::rad2deg(goal->angle / 2.0);
        int step_num = ceil(theta_rad / 0.02);
        int step_cnt = 0;

        geometry_msgs::TransformStamped transformStamped;
        try
        {
            transformStamped = tfBuffer_.lookupTransform("map", "base_link", ros::Time(0));
            initialYaw = tf2::getYaw(transformStamped.transform.rotation);
            ROS_INFO_STREAM("Initial yaw: " << initialYaw);
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
            // transformStamped = tfBuffer_.lookupTransform("map", "base_link", ros::Time(0));

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
                // double currentYaw = tf2::getYaw(transformStamped.transform.rotation);
                // // PublishFeedback(currentYaw);
                // ROS_INFO_STREAM("Current yaw: " << currentYaw);
                // double rot = FOLLOWING::rad2deg(currentYaw - initialYaw);
                // double therr = FOLLOWING::deg2rad(theta - rot);
                // if (!FOLLOWING::IsDoubleEqualtoZero(therr, 0.1))
                // {
                //     double w = rot_pid_controller_ptr_->calc_output(-therr, control_dt_) * 2.5;
                //     double velYaw = w / 2.0;
                //     geometry_msgs::Twist vel;
                //     vel.angular.z = velYaw;
                //     cmdVelPub_.publish(vel);
                // }
                // else
                // {
                //     geometry_msgs::Twist vel;
                //     vel.angular.z = 0.0;
                //     cmdVelPub_.publish(vel);
                //     state = false;
                //     initialYaw = currentYaw;
                //     progress++;
                // }

                if (step_cnt != step_num)
                {
                    geometry_msgs::Twist vel;
                    vel.angular.z = 0.2;
                    cmdVelPub_.publish(vel);
                    step_cnt++;
                }
                else
                {
                    geometry_msgs::Twist vel;
                    vel.angular.z = 0.0;
                    cmdVelPub_.publish(vel);
                    state = false;
                    step_cnt = 0;
                    progress++;
                }
            }
            else
            {
                // double currentYaw = tf2::getYaw(transformStamped.transform.rotation);
                // // PublishFeedback(currentYaw);
                // double rot = FOLLOWING::rad2deg(currentYaw - initialYaw);
                // double therr = FOLLOWING::deg2rad(rot - goal->angle);
                // if (!FOLLOWING::IsDoubleEqualtoZero(therr, 0.1))
                // {
                //     double w = rot_pid_controller_ptr_->calc_output(-therr, control_dt_) * 2.5;
                //     double velYaw = w / 2.0;
                //     geometry_msgs::Twist vel;
                //     vel.angular.z = velYaw;
                //     cmdVelPub_.publish(vel);
                // }
                // else
                // {
                //     geometry_msgs::Twist vel;
                //     vel.angular.z = 0.0;
                //     cmdVelPub_.publish(vel);
                //     state = true;
                //     initialYaw = currentYaw;
                //     progress++;
                // }

                if (step_cnt != 2 * step_num)
                {
                    geometry_msgs::Twist vel;
                    vel.angular.z = -0.2;
                    cmdVelPub_.publish(vel);
                    step_cnt++;
                }
                else
                {
                    geometry_msgs::Twist vel;
                    vel.angular.z = 0.0;
                    cmdVelPub_.publish(vel);
                    state = true;
                    step_cnt = 0;
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

    void LookforTargetServer::PublishFeedback(double current_yaw)
    {
        feedback_.current_yaw = current_yaw;
        as_.publishFeedback(feedback_);
    }
}