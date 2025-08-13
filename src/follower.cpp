#include <Eigen/Dense>
#include <tf/tf.h>
#include "following_controller/follower.h"

namespace FOLLOWING
{
    Follower::Follower(ros::NodeHandle nh) : nh_(nh), local_nh_("~"), tf_listener_(tf_buffer_), laserSub_(nh_, "scan_master", 100), odomSub_(nh_, "/odom", 100), targetSub_(nh_, "/mono_following/target", 100), is_navigating_(false), has_tried_lookfor_target_(false), scale_vel_x_(2.0), scale_vel_yaw_(2.5), ac_("move_base", true)
    {
        load_params();
        cmdVelPub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_x", 1);
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(100), laserSub_, targetSub_);
        sync_->registerCallback(std::bind(&Follower::TargetCallback, this, std::placeholders::_1, std::placeholders::_2));

        double rate = 10;
        double control_dt_ = 1.0 / rate;
        xy_pid_controller_ptr_ = std::make_unique<PID_controller>(0.3, 0.0, 0.1, 0.0, -max_vel_x_, max_vel_x_, -0.1, 0.1, control_dt_);
        th_pid_controller_ptr_ = std::make_unique<PID_controller>(1.0, 0.5, 0.2, 0.0, -max_vel_yaw_, max_vel_yaw_, -0.2, 0.2, control_dt_);

        lookfor_target_client_ptr_ = std::make_shared<LookforTargetClient>(nh_, "lookfor_target_action");
        lookfor_target_server_ptr_ = std::make_unique<LookforTargetServer>(nh_, "lookfor_target_action");

        Init();
        ROS_INFO_STREAM("Following controller is ready!");
    }

    Follower::~Follower() {}

    void Follower::Init()
    {
        targetInBase_.header.frame_id = "base_link";
        targetInBase_.pose.position.x = 0.0;
        targetInBase_.pose.position.y = 0.0;
        targetInBase_.pose.position.z = 0.0;
        targetInBase_.pose.orientation.x = 0.0;
        targetInBase_.pose.orientation.y = 0.0;
        targetInBase_.pose.orientation.z = 0.0;
        targetInBase_.pose.orientation.w = 1.0;

        targetInMap_.header.frame_id = "map";
        targetInMap_.pose.position.x = 0.0;
        targetInMap_.pose.position.y = 0.0;
        targetInMap_.pose.position.z = 0.0;
        targetInMap_.pose.orientation.x = 0.0;
        targetInMap_.pose.orientation.y = 0.0;
        targetInMap_.pose.orientation.z = 0.0;
        targetInMap_.pose.orientation.w = 1.0;

        pid_vel_.linear.x = 0.0;
        pid_vel_.angular.z = 0.0;

        cmd_vel_.linear.x = 0.0;
        cmd_vel_.angular.z = 0.0;

        last_pid_time_ = ros::Time::now();
        curr_pid_time_ = ros::Time::now();
    }

    void Follower::GetTargetInBase(const spencer_tracking_msgs::TargetPerson &targetMsg)
    {
        targetInBase_.pose.position.x = targetMsg.pose.pose.position.x;
        targetInBase_.pose.position.y = targetMsg.pose.pose.position.y;
        targetInBase_.pose.orientation.x = targetMsg.pose.pose.orientation.x;
        targetInBase_.pose.orientation.y = targetMsg.pose.pose.orientation.y;
        targetInBase_.pose.orientation.z = targetMsg.pose.pose.orientation.z;
        targetInBase_.pose.orientation.w = targetMsg.pose.pose.orientation.w;
    }

    void Follower::CreateObsList(const sensor_msgs::LaserScan::ConstPtr &scan)
    {
        obsList_.poses.clear();
        float angle = scan->angle_min;
        const int angle_index_step = static_cast<int>(scan_angle_resolution_ / scan->angle_increment);
        geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform("base_link", scan->header.frame_id, ros::Time(0), ros::Duration(0.1));

        for (int i = 0; i < scan->ranges.size(); i++)
        {
            const float range = scan->ranges[i];
            if (range < scan->range_min || range > scan->range_max || i % angle_index_step != 0)
            {
                angle += scan->angle_increment;
                continue;
            }

            geometry_msgs::PointStamped obs_lidar;
            obs_lidar.header.stamp = scan->header.stamp;
            obs_lidar.header.frame_id = scan->header.frame_id;
            obs_lidar.point.x = range * std::cos(angle);
            obs_lidar.point.y = range * std::sin(angle);
            obs_lidar.point.z = 0.0;

            geometry_msgs::PointStamped obs_base;
            tf2::doTransform(obs_lidar, obs_base, transform);

            geometry_msgs::Pose pose;
            pose.position = obs_base.point;
            obsList_.poses.push_back(pose);
            angle += scan->angle_increment;
        }
    }

    void Follower::CalcPIDVel(const spencer_tracking_msgs::TargetPerson &targetMsg)
    {
        double px = targetMsg.pose.pose.position.x;
        double py = targetMsg.pose.pose.position.y;
        double rx = distance_;
        double ry = 0.0;
        double th_err = std::atan2(py, px);
        double p_err = px - rx;
        double dt = (curr_pid_time_ - last_pid_time_).toSec();
        control_dt_ = dt > 0.1 ? 0.1 : dt;
        double w = th_pid_controller_ptr_->calc_output(-th_err, control_dt_) * scale_vel_yaw_;
        double v = xy_pid_controller_ptr_->calc_output(-p_err, control_dt_) * scale_vel_x_;
        double vx = 0.0;
        double vyaw = w / 2.0;

        const double angle_threshold = M_PI / 4.0;
        if (std::fabs(th_err) < angle_threshold)
        {
            double min_vel_x = enable_back_ ? -max_vel_x_ : 0.0;
            vx = std::clamp(v, min_vel_x, max_vel_x_);

            pid_vel_.linear.x = vx;
            pid_vel_.angular.z = vyaw;
        }
        else
        {
            ROS_INFO_STREAM("Rotation too big");
        }
    }

    void Follower::MoveOneStep(State &state, const double vel_x, const double vel_yaw)
    {
        const double sim_time_step = predict_time_ / static_cast<double>(sim_time_samples_);
        state.yaw_ += vel_yaw * sim_time_step;
        state.x_ += vel_x * std::cos(state.yaw_) * sim_time_step;
        state.y_ += vel_x * std::sin(state.yaw_) * sim_time_step;
        state.vel_x_ = vel_x;
        state.vel_yaw_ = vel_yaw;
    }

    std::vector<State> Follower::GenerateTrajectory(const double vel_x, const double vel_yaw)
    {
        std::vector<State> trajectory;
        trajectory.resize(sim_time_samples_);
        State state;
        for (int i = 0; i < sim_time_samples_; i++)
        {
            MoveOneStep(state, vel_x, vel_yaw);
            trajectory[i] = state;
        }
        return trajectory;
    }

    geometry_msgs::Pose Follower::TransformPoseInBaseToOdom(const nav_msgs::Odometry::ConstPtr &currentOdom, const geometry_msgs::Pose &poseInBase)
    {
        geometry_msgs::TransformStamped base_to_odom;
        base_to_odom.header.frame_id = "odom";
        base_to_odom.child_frame_id = "base_link";
        base_to_odom.header.stamp = ros::Time::now();

        base_to_odom.transform.translation.x = currentOdom->pose.pose.position.x;
        base_to_odom.transform.translation.y = currentOdom->pose.pose.position.y;
        base_to_odom.transform.translation.z = currentOdom->pose.pose.position.z;

        base_to_odom.transform.rotation = currentOdom->pose.pose.orientation;

        geometry_msgs::Pose poseInOdom;
        try
        {
            tf2::doTransform(poseInBase, poseInOdom, base_to_odom);
        }
        catch (const tf2::TransformException &ex)
        {
            ROS_ERROR_STREAM("Transform failed: " << ex.what());
            return geometry_msgs::Pose();
        }

        return poseInOdom;
    }

    geometry_msgs::Pose Follower::TransformPoseInBaseToMap(const geometry_msgs::Pose &poseInBase)
    {
        std::string map_frame = "map";
        std::string base_frame = "base_link";

        geometry_msgs::PoseStamped base_pose;
        geometry_msgs::PoseStamped map_pose;

        base_pose.header.frame_id = base_frame;
        base_pose.header.stamp = ros::Time::now();
        base_pose.pose = poseInBase;

        try
        {
            auto transform = tf_buffer_.lookupTransform(map_frame, base_frame, ros::Time(0), ros::Duration(1.0));

            tf_buffer_.transform(base_pose, map_pose, map_frame, ros::Duration(1.0));
        }
        catch (const std::exception &e)
        {
            ROS_ERROR("Transform error: %s", e.what());
        }

        return map_pose.pose;
    }

    geometry_msgs::PolygonStamped Follower::MoveFootprint(const geometry_msgs::Pose &goalInBase, const spencer_tracking_msgs::TargetPerson &targetMsg)
    {
        geometry_msgs::PolygonStamped moved_footprint;

        double dx = targetMsg.pose.pose.position.x - goalInBase.position.x;
        double dy = targetMsg.pose.pose.position.y - goalInBase.position.y;
        double yaw = std::atan2(dy, dx);

        for (auto &point : footprint_.polygon.points)
        {
            Eigen::VectorXf point_in(2);
            point_in << point.x, point.y;
            Eigen::Matrix2f rotation;
            rotation = Eigen::Rotation2Df(yaw);
            Eigen::VectorXf point_out(2);
            point_out = rotation * point_in;

            geometry_msgs::Point32 moved_point;
            moved_point.x = point_out.x() + goalInBase.position.x;
            moved_point.y = point_out.y() + goalInBase.position.y;

            moved_footprint.polygon.points.push_back(moved_point);
        }

        return moved_footprint;
    }

    geometry_msgs::PolygonStamped Follower::MoveFootprint(const State &step)
    {
        geometry_msgs::PolygonStamped moved_footprint;

        for (auto &point : footprint_.polygon.points)
        {
            Eigen::VectorXf point_in(2);
            point_in << point.x, point.y;
            Eigen::Matrix2f rotation;
            rotation = Eigen::Rotation2Df(step.yaw_);
            Eigen::VectorXf point_out(2);
            point_out = rotation * point_in;

            geometry_msgs::Point32 moved_point;
            moved_point.x = point_out.x() + step.x_;
            moved_point.y = point_out.y() + step.y_;

            moved_footprint.polygon.points.push_back(moved_point);
        }

        return moved_footprint;
    }

    bool Follower::CheckPointInTriangle(const geometry_msgs::Point &obsPoint, const geometry_msgs::Polygon &triangle)
    {
        if (triangle.points.size() != 3)
        {
            ROS_ERROR("Triangle must have 3 points");
            exit(1);
        }

        const Eigen::Vector3d point_A(triangle.points[0].x, triangle.points[0].y, 0.0);
        const Eigen::Vector3d point_B(triangle.points[1].x, triangle.points[1].y, 0.0);
        const Eigen::Vector3d point_C(triangle.points[2].x, triangle.points[2].y, 0.0);
        const Eigen::Vector3d point_Obs(obsPoint.x, obsPoint.y, 0.0);

        const Eigen::Vector3d vec_AB = point_B - point_A;
        const Eigen::Vector3d vec_BD = point_Obs - point_B;
        const Eigen::Vector3d cross_AB_BD = vec_AB.cross(vec_BD);

        const Eigen::Vector3d vec_BC = point_C - point_B;
        const Eigen::Vector3d vec_CD = point_Obs - point_C;
        const Eigen::Vector3d cross_BC_CD = vec_BC.cross(vec_CD);

        const Eigen::Vector3d vec_CA = point_A - point_C;
        const Eigen::Vector3d vec_AD = point_Obs - point_A;
        const Eigen::Vector3d cross_CA_AD = vec_CA.cross(vec_AD);

        if (cross_AB_BD.dot(cross_CA_AD) > 0 && cross_BC_CD.dot(cross_CA_AD) > 0)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    bool Follower::CheckPointInRobot(const geometry_msgs::Point &obsPoint, const geometry_msgs::PolygonStamped &footprint, const State &step)
    {
        geometry_msgs::Point32 point;
        point.x = step.x_;
        point.y = step.y_;

        for (int i = 0; i < footprint.polygon.points.size(); i++)
        {
            geometry_msgs::Polygon triangle;
            triangle.points.push_back(point);
            triangle.points.push_back(footprint.polygon.points[i]);
            if (i == footprint.polygon.points.size() - 1)
            {
                triangle.points.push_back(footprint.polygon.points[0]);
            }
            else
            {
                triangle.points.push_back(footprint.polygon.points[i + 1]);
            }

            if (CheckPointInTriangle(obsPoint, triangle))
            {
                return true;
            }
        }

        return false;
    }

    bool Follower::CheckCollision(const std::vector<State> &trajectory)
    {
        for (auto &step : trajectory)
        {
            for (auto &obs : obsList_.poses)
            {
                const geometry_msgs::PolygonStamped moved_footprint = MoveFootprint(step);
                if (CheckPointInRobot(obs.position, moved_footprint, step))
                {
                    return true;
                }
            }
        }
        return false;
    }

    void
    Follower::TargetCallback(const sensor_msgs::LaserScan::ConstPtr &laserMsg, const spencer_tracking_msgs::TargetPerson::ConstPtr &targetMsg)
    {
        curr_pid_time_ = ros::Time::now();
        std::lock_guard<std::mutex> lock(nav_mutex_);
        if (is_navigating_)
        {
            ROS_INFO_THROTTLE(1.0, "Status: Navigating");
            return;
        }

        spencer_tracking_msgs::TargetPerson target_msg;
        target_msg = *targetMsg;

        if (FOLLOWING::IsDoubleEqualtoZero(target_msg.pose.pose.position.x))
        {
            ROS_WARN_STREAM("No target is found!");
            if (!lookfor_target_client_ptr_->GetState() && !has_tried_lookfor_target_)
            {
                ROS_INFO("Start lookfor_target_action");
                lookfor_target_action::LookforTargetActionGoal goal;
                goal.angle = 30.0;
                lookfor_target_client_ptr_->SendGoal(goal);
                has_tried_lookfor_target_ = true;
            }
            return;
        }
        else
        {
            if (lookfor_target_client_ptr_->GetState())
            {
                lookfor_target_client_ptr_->CancelGoal();
            }
            has_tried_lookfor_target_ = false;
        }

        if (target_msg.pose.pose.position.x > 3.5)
        {
            ROS_WARN_STREAM("Target is too far away!");
            return;
        }

        GetTargetInBase(target_msg);
        CalcPIDVel(target_msg);
        CreateObsList(laserMsg);

        double vx = pid_vel_.linear.x;
        double vyaw = pid_vel_.angular.z;
        std::vector<State> trajectory = GenerateTrajectory(vx, vyaw);

        if (!CheckCollision(trajectory))
        {
            cmd_vel_.linear.x = vx;
            cmd_vel_.angular.z = vyaw;
            cmdVelPub_.publish(cmd_vel_);
            last_pid_time_ = curr_pid_time_;
            ROS_INFO_STREAM("Execute PID velocity command: vx: " << vx << ", vyaw: " << vyaw);
        }
        else
        {
            is_navigating_ = true;
            targetInMap_.pose = TransformPoseInBaseToMap(targetInBase_.pose);
            move_base_msgs::MoveBaseGoal goal;
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose.position.x = targetInMap_.pose.position.x;
            goal.target_pose.pose.position.y = targetInMap_.pose.position.y;
            goal.target_pose.pose.orientation = target_msg.pose.pose.orientation;

            if (!ac_.waitForServer(ros::Duration(5.0)))
            {
                ROS_ERROR_STREAM("Failed to connect to move_base server!");
                is_navigating_ = false;
                return;
            }
            ROS_INFO_STREAM("Send goal to move_base: x: " << goal.target_pose.pose.position.x << " y: " << goal.target_pose.pose.position.y);
            ac_.sendGoal(goal,
                         [this](const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result)
                         {
                             std::lock_guard<std::mutex> lock(nav_mutex_);
                             is_navigating_ = false;
                             ROS_INFO_STREAM("Move_base state: " << state.toString());
                         });
        }
    }
}