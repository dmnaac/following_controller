#include <Eigen/Dense>
#include <tf/tf.h>
#include "following_controller/follower.h"

namespace FOLLOWING
{
    Follower::Follower(ros::NodeHandle nh) : nh_(nh), local_nh_("~"), laserSub_(nh_, "scan", 100), odomSub_(nh_, "odom", 100), targetSub_(nh_, "/mono_following/target", 100)
    {
        local_nh_.param<double>("DISTANCE", distance_, 1.0);
        goalPub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy, laserSub_, odomSub_, targetSub_);
        sync_->registerCallback(std::bind(&Follower::TargetCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        InitPose();
    }

    Follower::~Follower() {}

    void Follower::InitPose()
    {
        targetInBase_.header.frame_id = "base_link";
        targetInBase_.pose.position.x = 0.0;
        targetInBase_.pose.position.y = 0.0;
        targetInBase_.pose.position.z = 0.0;
        targetInBase_.pose.orientation.x = 0.0;
        targetInBase_.pose.orientation.y = 0.0;
        targetInBase_.pose.orientation.z = 0.0;
        targetInBase_.pose.orientation.w = 1.0;

        bestGoalInOdom_.header.frame_id = "odom";
        bestGoalInOdom_.pose.position.x = 0.0;
        bestGoalInOdom_.pose.position.y = 0.0;
        bestGoalInOdom_.pose.position.z = 0.0;
        bestGoalInOdom_.pose.orientation.x = 0.0;
        bestGoalInOdom_.pose.orientation.y = 0.0;
        bestGoalInOdom_.pose.orientation.z = 0.0;
        bestGoalInOdom_.pose.orientation.w = 1.0;
    }

    void Follower::CreateObsList(const sensor_msgs::LaserScan::ConstPtr &scan)
    {
        obsList_.poses.clear();
        float angle = scan->angle_min;
        const int angle_index_step = static_cast<int>(scan_angle_resolution_ / scan->angle_increment);
        geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform("base_link", scan->header.framed_id, ros::Time(0), ros::Duration(0.1));

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

    geometry_msgs::Pose Follower::TransformBestGoalToOdom(const nav_msgs::Odometry::ConstPtr &currentOdom, const geometry_msgs::Pose &bestGoalInBase)
    {
        geometry_msgs::Pose robotPoseInOdom = currentOdom->pose.pose;
        tf::Quaternion odom_quat(robotPoseInOdom.orientation.x,
                                 robotPoseInOdom.orientation.y,
                                 robotPoseInOdom.orientation.z,
                                 robotPoseInOdom.orientation.w);

        tf::Matrix3x3 odom_rot(odom_quat);

        tf::Quaternion goal_quat(bestGoalInBase.orientation.x,
                                 bestGoalInBase.orientation.y,
                                 bestGoalInBase.orientation.z,
                                 bestGoalInBase.orientation.w);

        tf::Matrix3x3 goal_rot(goal_quat);

        tf::Matrix3x3 final_rot = odom_rot * goal_rot;
        tf::Quaternion final_quat;
        final_rot.getRotation(final_quat);

        tf::Vector3 goal_pos(bestGoalInBase.position.x,
                             bestGoalInBase.position.y,
                             bestGoalInBase.position.z);
        tf::Vector3 rotated_goal_pos = odom_rot * goal_pos;

        tf::Vector3 final_pos(rotated_goal_pos.x() + robotPoseInOdom.position.x,
                              rotated_goal_pos.y() + robotPoseInOdom.position.y,
                              rotated_goal_pos.z() + robotPoseInOdom.position.z);

        geometry_msgs::Pose bestGoalInOdom;
        bestGoalInOdom.position.x = final_pos.x();
        bestGoalInOdom.position.y = final_pos.y();
        bestGoalInOdom.position.z = final_pos.z();
        bestGoalInOdom.orientation.x = final_quat.x();
        bestGoalInOdom.orientation.y = final_quat.y();
        bestGoalInOdom.orientation.z = final_quat.z();
        bestGoalInOdom.orientation.w = final_quat.w();

        return bestGoalInOdom;
    }

    /**
     * @brief Generate goal samples around the target position in the base frame.
     *
     * This function generates a specified number of goal poses arranged in a semicircle
     * around the target's position in the base frame, at a fixed distance.
     *
     * @param numOfSamples Number of goal samples to generate.
     */
    void Follower::GenerateGoalSamplesInBase(int numOfSamples)
    {
        int num = numOfSamples;
        for (int i = 1; i < num; i++)
        {
            geometry_msgs::Pose pose;
            double angle = M_PI * (i / num + 1);
            double dx = distance_ * std::cos(angle);
            double dy = distance_ * std::sin(angle);
            pose.position.x = targetInBase_.pose.position.x + dx;
            pose.position.y = targetInBase_.pose.position.y + dy;
            double yaw = std::atan2(-dy, -dx);
            geometry_msgs::Quaternion q;
            q.x = 0.0;
            q.y = 0.0;
            q.z = std::sin(yaw / 2);
            q.w = std::cos(yaw / 2);
            pose.orientation = q;
            goalSamplesInBase_.poses.push_back(pose);
        }
    }

    geometry_msgs::PolygonStamped Follower::MoveFootprint(const geometry_msgs::Pose goalInBase, const spencer_tracking_msgs::TargetPerson targetMsg)
    {
        geometry_msgs::PolygonStamped moved_footprint;

        double dx = goalInBase.position.x - targetMsg.pose.pose.position.x;
        double dy = goalInBase.position.y - targetMsg.pose.pose.position.y;
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

    bool Follower::CheckPointInRobot(const geometry_msgs::Point &obsPoint, const geometry_msgs::PolygonStamped &footprint, const geometry_msgs::Pose &goalInBase)
    {
        geometry_msgs::Point32 point;
        point.x = goalInBase.position.x;
        point.y = goalInBase.position.y;

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
    }

    bool Follower::CheckCollision(const geometry_msgs::Pose &goalInBase, const spencer_tracking_msgs::TargetPerson &targetMsg)
    {
        geometry_msgs::PolygonStamped moved_footprint = MoveFootprint(goalInBase, targetMsg);
        for (auto &obs : obsList_.poses)
        {
            if (CheckPointInRobot(obs.position, moved_footprint))
            {
                return true;
            }
        }
        return false;
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

    void
    Follower::TargetCallback(const sensor_msgs::LaserScan::ConstPtr &laserMsg, const nav_msgs::Odometry::ConstPtr &odomMsg, const spencer_tracking_msgs::TrackedPersons::ConstPtr &targetMsg)
    {
        spencer_tracking_msgs::TargetPerson target_msg;
        target_msg = *targetMsg;

        GetTargetInBase(target_msg);
        CreateObsList(laserMsg);
        GenerateGoalSamplesInBase(10);

        std::vector<geometry_msgs::Pose> goalsInBase;
        for (auto &goal : goalSamplesInBase_.poses)
        {
            if (!CheckCollision(goal, target_msg))
            {
                goalsInBase.push_back(goal);
            }
        }

        geometry_msgs::Pose bestGoalInBase;
        double minDist = 10000.0;
        for (auto &goal : goalsInBase)
        {
            double dist = std::sqrt(std::pow(goal.position.x, 2) + std::pow(goal.position.y, 2));
            if (dist < minDist)
            {
                minDist = dist;
                bestGoalInBase = goal;
            }
        }

        bestGoalInOdom_.header.stamp = ros::Time::now();
        bestGoalInOdom_.pose = TransformBestGoalToOdom(odomMsg, bestGoalInBase);
        goalPub_.publish(bestGoalInOdom_);
    }
}