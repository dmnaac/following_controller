#include <algorithm>
#include <string>

#include "following_controller/follower.h"

namespace FOLLOWING
{
    double getNumberFromXMLRPC(XmlRpc::XmlRpcValue &value, const std::string &full_param_name)
    {
        // Make sure that the value we're looking at is either a double or an int.
        if (value.getType() != XmlRpc::XmlRpcValue::TypeInt &&
            value.getType() != XmlRpc::XmlRpcValue::TypeDouble)
        {
            std::string &value_string = value;
            ROS_FATAL("Values in the footprint specification (param %s) must be numbers. Found value %s.",
                      full_param_name.c_str(), value_string.c_str());
            throw std::runtime_error("Values in the footprint specification must be numbers");
        }
        return value.getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(value) : (double)(value);
    }

    std::vector<geometry_msgs::Point32> makeFootprintFromXMLRPC(XmlRpc::XmlRpcValue &footprint_xmlrpc,
                                                                const std::string &full_param_name)
    {
        // Make sure we have an array of at least 3 elements.
        if (footprint_xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeArray ||
            footprint_xmlrpc.size() < 3)
        {
            ROS_FATAL("The footprint must be specified as list of lists on the parameter server, %s was specified as %s",
                      full_param_name.c_str(), std::string(footprint_xmlrpc).c_str());
            throw std::runtime_error("The footprint must be specified as list of lists on the parameter server with at least "
                                     "3 points eg: [[x1, y1], [x2, y2], ..., [xn, yn]]");
        }

        std::vector<geometry_msgs::Point32> footprint;
        geometry_msgs::Point32 pt;

        for (int i = 0; i < footprint_xmlrpc.size(); ++i)
        {
            // Make sure each element of the list is an array of size 2. (x and y coordinates)
            XmlRpc::XmlRpcValue point = footprint_xmlrpc[i];
            if (point.getType() != XmlRpc::XmlRpcValue::TypeArray ||
                point.size() != 2)
            {
                ROS_FATAL("The footprint (parameter %s) must be specified as list of lists on the parameter server eg: "
                          "[[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form.",
                          full_param_name.c_str());
                throw std::runtime_error("The footprint must be specified as list of lists on the parameter server eg: "
                                         "[[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form");
            }

            pt.x = getNumberFromXMLRPC(point[0], full_param_name);
            pt.y = getNumberFromXMLRPC(point[1], full_param_name);

            footprint.push_back(pt);
        }
        return footprint;
    }

    void Follower::load_params()
    {
        local_nh_.param<double>("DISTANCE", distance_, 1.0);
        local_nh_.param<double>("SCAN_ANGLE_RESOLUTION", scan_angle_resolution_, 0.087);
        local_nh_.param<double>("FOOTPRINT_PADDING", footprint_padding_, 0.0);

        XmlRpc::XmlRpcValue footprint_xmlrpc;
        if (!local_nh_.getParam("FOOTPRINT", footprint_xmlrpc))
        {
            ROS_ERROR("USE_FOOTPRINT but not found FOOTPRINT param");
        }
        else
        {
            footprint_points_ = makeFootprintFromXMLRPC(footprint_xmlrpc, "FOOTPRINT");
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