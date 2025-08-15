#include "following_controller/lookfortargetserver.h"

namespace FOLLOWING
{
    LookforTargetServer::LookforTargetServer(ros::NodeHandle &nh, const std::string &action_name) : nh_(nh), as_(nh, action_name, boost::bind(&LookforTargetServer::ExecuteCB, this, _1), false), tfListener_(tfBuffer_), action_name_(action_name), is_active_(false), current_yaw_(0.0), start_yaw_(0.0), angle_tolerance_(0.08)
    {
        cmdVelPub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel_x", 10);
        imuSub_ = nh_.subscribe("imu", 10, &LookforTargetServer::ImuCallback, this);
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

    void LookforTargetServer::ImuCallback(const sensor_msgs::Imu::ConstPtr &msg)
    {
        tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        {
            std::lock_guard<std::mutex> lock(yaw_mutex_);
            current_yaw_ = yaw;
        }
    }

    void LookforTargetServer::ExecuteCB(const lookfor_target_action::LookforTargetGoalConstPtr &goal)
    {
        ROS_INFO("Start execution");

        bool success = false;
        is_active_ = true;

        double theta_rad = FOLLOWING::deg2rad(goal->angle / 2.0);

        bool step1_success = RotationControl(theta_rad);
        bool step2_success = RotationControl(-2 * theta_rad);
        bool step3_success = RotationControl(theta_rad);

        if (step1_success && step2_success && step3_success)
        {
            result_.success = true;
            result_.message = "lookfor_target_action completed";
            as_.setSucceeded(result_);
        }
        else
        {
            result_.success = false;
            result_.message = "lookfor_target_action not completed";
            as_.setAborted(result_);
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

        {
            std::lock_guard<std::mutex> lock(yaw_mutex_);
            start_yaw_ = current_yaw_;
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

            double current_yaw;
            {
                std::lock_guard<std::mutex> lock(yaw_mutex_);
                current_yaw = current_yaw_;
            }
            double rotated_angle = current_yaw - start_yaw_;
            rotated_angle = FOLLOWING::NormalizeAngle(rotated_angle);
            double error = angle - rotated_angle;

            if (fabs(error) < angle_tolerance_)
            {
                vel.linear.x = 0.0;
                vel.angular.z = 0.0;
                cmdVelPub_.publish(vel);
                break;
            }

            double w = rot_pid_controller_ptr_->calc_output(-error, control_dt_);
            double velYaw = w * 1.25;
            vel.linear.x = 0.0;
            vel.angular.z = velYaw;
            cmdVelPub_.publish(vel);

            rate.sleep();
        }

        return true;
    }
}