#include "following_controller/lookfortargetserver.h"

namespace FOLLOWING
{
    LookforTargetServer::LookforTargetServer(ros::NodeHandle &nh, const std::string &action_name, double KP, double KI, double KD) : nh_(nh), as_(nh, action_name, boost::bind(&LookforTargetServer::ExecuteCB, this, _1), false), tfListener_(tfBuffer_), action_name_(action_name), is_active_(false), current_yaw_(0.0), start_yaw_(0.0), angle_tolerance_(0.08), kp_(KP), ki_(KI), kd_(KD)
    {
        cmdVelPub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel_x", 10);
        double rate = 10;
        double control_dt_ = 1.0 / rate;
        rot_pid_controller_ptr_ = std::make_unique<PID_controller>(kp_, ki_, kd_, 0.0, -0.2, 0.2, -1.0, 1.0, control_dt_, "Rotation");

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
        ROS_INFO("Wait 5 seconds and start execution");
        ros::Duration(5.0).sleep();

        bool success = false;
        is_active_ = true;

        double theta_rad = FOLLOWING::deg2rad(goal->angle / 2.0);

        bool step1_success = RotationControl(theta_rad);
        ros::Duration(2.0).sleep();
        bool step2_success = RotationControl(-2.0 * theta_rad);
        ros::Duration(2.0).sleep();
        bool step3_success = RotationControl(theta_rad);

        if (step1_success && step2_success && step3_success)
        {
            result_.success = true;
            result_.message = "lookfor_target_action completed";
            as_.setSucceeded(result_);
        }
    }

    void LookforTargetServer::PublishFeedback(double current_yaw)
    {
        feedback_.current_yaw = current_yaw;
        as_.publishFeedback(feedback_);
    }

    bool LookforTargetServer::RotationControl(double angle)
    {
        if (!is_active_)
        {
            return false;
        }

        geometry_msgs::TransformStamped transformStamped;
        try
        {
            transformStamped = tfBuffer_.lookupTransform("map", "base_link", ros::Time(0));
            start_yaw_ = tf2::getYaw(transformStamped.transform.rotation);
        }
        catch (const std::exception &ex)
        {
            ROS_ERROR("Failed to lookup transform: %s", ex.what());
            result_.success = false;
            result_.message = "Failed to lookup transform";
            as_.setAborted(result_);
            is_active_ = false;
            return false;
        }

        ros::Rate rate(10);
        geometry_msgs::Twist vel;

        while (ros::ok())
        {
            if (as_.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO_STREAM("Action preempted: " << action_name_.c_str());
                result_.success = false;
                result_.message = "Action preempted";
                as_.setPreempted(result_);
                is_active_ = false;
                return false;
            }

            transformStamped = tfBuffer_.lookupTransform("map", "base_link", ros::Time(0));
            double current_yaw_ = tf2::getYaw(transformStamped.transform.rotation);

            double rotated_angle = current_yaw_ - start_yaw_;
            rotated_angle = FOLLOWING::NormalizeAngle(rotated_angle);
            double error = angle - rotated_angle;
            error = FOLLOWING::NormalizeAngle(error);

            // PublishFeedback(currentYaw);

            if (fabs(error) < angle_tolerance_)
            {
                vel.linear.x = 0.0;
                vel.angular.z = 0.0;
                cmdVelPub_.publish(vel);
                break;
            }

            double w = rot_pid_controller_ptr_->calc_output(-error, 0.1);
            vel.linear.x = 0.0;
            vel.angular.z = w;
            cmdVelPub_.publish(vel);
            ROS_INFO_STREAM("YAW: " << w);

            rate.sleep();
        }

        return true;
    }
}