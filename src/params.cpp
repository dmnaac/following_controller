#include <algorithm>
#include <string>

#include "following_controller/follower.h"
#include "following_controller/utils.h"

namespace FOLLOWING
{
    void Follower::load_params()
    {
        local_nh_.param<double>("DISTANCE", distance_, 1.0);
        local_nh_.param<double>("SCAN_ANGLE_RESOLUTION", scan_angle_resolution_, 0.087);
        local_nh_.param<double>("FOOTPRINT_PADDING", footprint_padding_, 0.0);

        local_nh_.param<double>("MAX_LINEAR_VEL", max_vel_x_, 0.2);
        local_nh_.param<double>("MAX_ANGULAR_VEL", max_vel_yaw_, 0.5);
        local_nh_.param<int>("SIM_TIME_SAMPLES", sim_time_samples_, 10);
        local_nh_.param<double>("PREDICT_TIME", predict_time_, 2.0);
        local_nh_.param<bool>("ENABLE_BACK", enable_back_, true);

        local_nh_.param<double>("KP", kP_, 0.25);
        local_nh_.param<double>("KI", kI_, 0.5);
        local_nh_.param<double>("KD", kD_, 0.1);

        local_nh_.param<double>("KP_LINEAR_VEL", kP_linear_vel_, 0.3);
        local_nh_.param<double>("KI_LINEAR_VEL", kI_linear_vel_, 0.0);
        local_nh_.param<double>("KD_LINEAR_VEL", kD_linear_vel_, 0.1);
        local_nh_.param<double>("KP_ANGULAR_VEL", kP_angular_vel_, 1.0);
        local_nh_.param<double>("KI_ANGULAR_VEL", kI_angular_vel_, 0.5);
        local_nh_.param<double>("KD_ANGULAR_VEL", kD_angular_vel_, 0.2);
        local_nh_.param<double>("DISTANCE_TOLERANCE", distance_tolerance_, 0.1);
        local_nh_.param<double>("ANGLE_TOLERANCE", angle_tolerance_, 0.08);

        XmlRpc::XmlRpcValue footprint_xmlrpc;
        if (!local_nh_.getParam("FOOTPRINT", footprint_xmlrpc))
        {
            ROS_ERROR("USE_FOOTPRINT but not found FOOTPRINT param");
        }
        else
        {
            footprint_points_ = FOLLOWING::makeFootprintFromXMLRPC(footprint_xmlrpc, "FOOTPRINT");
        }

        if (footprint_points_.size() < 3)
        {
            ROS_ERROR("Footprint must have at least 3 points");
        }
        else
        {
            std::vector<geometry_msgs::Point32> padded_points;
            for (int i = 0; i < footprint_points_.size(); i++)
            {
                geometry_msgs::Point32 point;
                int direction_x = (footprint_points_[i].x < 0) ? -1 : 1;
                point.x = footprint_points_[i].x + direction_x * footprint_padding_;
                int direction_y = (footprint_points_[i].y < 0) ? -1 : 1;
                point.y = footprint_points_[i].y + direction_y * footprint_padding_;
                padded_points.push_back(point);
            }

            int num_segments = 10;
            for (int i = 0; i < padded_points.size() - 1; i++)
            {
                for (int j = 1; j < num_segments; j++)
                {
                    // The first point is the same as the padded_points_[i]
                    // When this point is the first, it will not be added to the footprint, therefore j starts from 1

                    double t = static_cast<double>(j) / num_segments;
                    geometry_msgs::Point32 point;
                    point.x = padded_points[i].x + t * (padded_points[i + 1].x - padded_points[i].x);
                    point.y = padded_points[i].y + t * (padded_points[i + 1].y - padded_points[i].y);
                    footprint_.polygon.points.push_back(point);
                }
            }
            // Process the last padded footprint point
            for (int j = 1; j < num_segments; j++)
            {
                double t = static_cast<double>(j) / num_segments;
                geometry_msgs::Point32 point;
                int point_index = padded_points.size() - 1;
                point.x = padded_points[point_index].x + t * (padded_points[0].x - padded_points[point_index].x);
                point.y = padded_points[point_index].y + t * (padded_points[0].y - padded_points[point_index].y);
                footprint_.polygon.points.push_back(point);
            }

            footprint_.header.stamp = ros::Time::now();
            footprint_.header.frame_id = "base_link";
        }
    }
}