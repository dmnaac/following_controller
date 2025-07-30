#ifndef FOLLOWING_CONTROLLER_FOLLOWER_H
#define FOLLOWING_CONTROLLER_FOLLOWER_H

#include <memory>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PolygonStamped.h>

#include <spencer_tracking_msgs/TargetPerson.h>

namespace FOLLOWING
{
    class Follower
    {
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle local_nh_;
        ros::Publisher goalPub_;

        message_filters::Subscriber<sensor_msgs::LaserScan> laserSub_;
        message_filters::Subscriber<nav_msgs::Odometry> odomSub_;
        message_filters::Subscriber<spencer_tracking_msgs::TargetPerson> targetSub_;
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, nav_msgs::Odometry, spencer_tracking_msgs::TargetPerson> SyncPolicy;
        std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

        double distance_;
        double scan_angle_resolution_;
        double footprint_padding_;
        std::vector<geometry_msgs::Point32> footprint_points_;
        geometry_msgs::PolygonStamped footprint_;
        geometry_msgs::PoseStamped targetInBase_;
        geometry_msgs::PoseArray goalSamplesInBase_;
        geometry_msgs::PoseArray obsList_;
        geometry_msgs::PoseStamped bestGoalInOdom_;

        geometry_msgs::Pose TransformBestGoalToOdom(const nav_msgs::Odometry::ConstPtr &currentOdom, const geometry_msgs::Pose &bestGoalInBase);
        geometry_msgs::PolygonStamped MoveFootprint(const geometry_msgs::Pose &goalInBase, const spencer_tracking_msgs::TargetPerson &targetMsg);
        void CreateObsList(const sensor_msgs::LaserScan::ConstPtr &scan);
        bool CheckPointInTriangle(const geometry_msgs::Point &obsPoint, const geometry_msgs::Polygon &triangle);
        bool CheckPointInRobot(const geometry_msgs::Point &obsPoint, const geometry_msgs::PolygonStamped &footprint, const geometry_msgs::Pose &goalInBase);
        bool CheckCollision(const geometry_msgs::Pose &goalInBase, const spencer_tracking_msgs::TargetPerson &targetMsg);
        void GetTargetInBase(const spencer_tracking_msgs::TargetPerson &targetMsg);
        void GenerateGoalSamplesInBase(int numOfSamples);
        void InitPose();

    public:
        Follower(ros::NodeHandle nh);
        ~Follower();

        void TargetCallback(const sensor_msgs::LaserScan::ConstPtr &laserMsg, const nav_msgs::Odometry::ConstPtr &odomMsg, const spencer_tracking_msgs::TargetPerson::ConstPtr &targetMsg);

        void load_params();
    };
}

#endif // FOLLOWING_CONTROLLER_FOLLOWER_H