#ifndef UTILS_H
#define UTILS_H

#include <XmlRpcValue.h>
#include <ros/ros.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace FOLLOWING
{
    const double EPSILON = 1e-9;
    bool IsDoubleEqualtoZero(double value, double mark = EPSILON);
    double getNumberFromXMLRPC(XmlRpc::XmlRpcValue &value, const std::string &full_param_name);
    std::vector<geometry_msgs::Point32> makeFootprintFromXMLRPC(XmlRpc::XmlRpcValue &footprint_xmlrpc, const std::string &full_param_name);
    bool IsKeyTarget(geometry_msgs::Point last_target_pos, geometry_msgs::Point cur_target_pos);
    double rad2deg(double rad);
    double deg2rad(double deg);
    double NormalizeAngle(double angle);
    geometry_msgs::Quaternion InverseQuaternion(geometry_msgs::Pose pose);
}

#endif // UTILS_H