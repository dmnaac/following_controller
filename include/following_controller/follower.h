#ifndef FOLLOWING_CONTROLLER_FOLLOWER_H
#define FOLLOWING_CONTROLLER_FOLLOWER_H

#include <mutex>
#include <memory>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
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
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <spencer_tracking_msgs/TargetPerson.h>

#include "following_controller/lookfortargetclient.h"
#include "following_controller/lookfortargetserver.h"
#include "following_controller/pid_controller.h"
#include "following_controller/state.h"
#include "following_controller/utils.h"

namespace FOLLOWING
{
    class Follower
    {
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle local_nh_;
        ros::Publisher cmdVelPub_;

        message_filters::Subscriber<sensor_msgs::LaserScan> laserSub_;
        message_filters::Subscriber<nav_msgs::Odometry> odomSub_;
        message_filters::Subscriber<spencer_tracking_msgs::TargetPerson> targetSub_;
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, spencer_tracking_msgs::TargetPerson> SyncPolicy;
        std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

        bool is_navigating_;
        bool has_tried_lookfor_target_;
        std::mutex nav_mutex_;
        typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
        MoveBaseClient ac_;

        std::unique_ptr<LookforTargetClient> lookfor_target_client_ptr_;
        std::unique_ptr<LookforTargetServer> lookfor_target_server_ptr_;

        std::unique_ptr<PID_controller> xy_pid_controller_ptr_;
        std::unique_ptr<PID_controller> th_pid_controller_ptr_;
        bool enable_back_;
        double max_vel_x_;
        double max_vel_yaw_;
        double scale_vel_x_;
        double scale_vel_yaw_;
        double control_dt_;
        geometry_msgs::Twist pid_vel_;
        geometry_msgs::Twist cmd_vel_;
        int sim_time_samples_;
        double predict_time_;
        ros::Time last_pid_time_;
        ros::Time curr_pid_time_;
        double kP_;
        double kI_;
        double kD_;
        double kP_linear_vel_;
        double kI_linear_vel_;
        double kD_linear_vel_;
        double kP_angular_vel_;
        double kI_angular_vel_;
        double kD_angular_vel_;
        double distance_tolerance_;
        double angle_tolerance_;

        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

        double distance_;
        double scan_angle_resolution_;
        double footprint_padding_;
        std::vector<geometry_msgs::Point32> footprint_points_;
        geometry_msgs::PolygonStamped footprint_;
        geometry_msgs::PoseStamped targetInBase_;
        geometry_msgs::PoseStamped targetInMap_;
        geometry_msgs::PoseArray obsList_;
        geometry_msgs::Point last_target_pos_;

        geometry_msgs::Pose TransformPoseInBaseToOdom(const nav_msgs::Odometry::ConstPtr &currentOdom, const geometry_msgs::Pose &poseInBase);
        geometry_msgs::Pose TransformPoseInBaseToMap(const geometry_msgs::Pose &poseInBase);
        geometry_msgs::PolygonStamped MoveFootprint(const geometry_msgs::Pose &goalInBase, const spencer_tracking_msgs::TargetPerson &targetMsg);
        geometry_msgs::PolygonStamped MoveFootprint(const State &step);
        void CreateObsList(const sensor_msgs::LaserScan::ConstPtr &scan);
        bool CheckPointInTriangle(const geometry_msgs::Point &obsPoint, const geometry_msgs::Polygon &triangle);
        bool CheckPointInRobot(const geometry_msgs::Point &obsPoint, const geometry_msgs::PolygonStamped &footprint, const State &step);
        bool CheckCollision(const std::vector<State> &trajectory);
        void GetTargetInBase(const spencer_tracking_msgs::TargetPerson &targetMsg);
        void Init();
        void CalcPIDVel(const spencer_tracking_msgs::TargetPerson &targetMsg);
        std::vector<State> GenerateTrajectory(const double vel_x, const double vel_yaw);
        void MoveOneStep(State &state, const double vel_x, const double vel_yaw);

    public:
        Follower(ros::NodeHandle nh);
        ~Follower();

        void TargetCallback(const sensor_msgs::LaserScan::ConstPtr &laserMsg, const spencer_tracking_msgs::TargetPerson::ConstPtr &targetMsg);

        void load_params();
    };
}

#endif // FOLLOWING_CONTROLLER_FOLLOWER_H