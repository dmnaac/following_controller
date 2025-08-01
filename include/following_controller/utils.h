#ifndef UTILS_H
#define UTILS_H

#include "following_controller/follower.h"

namespace FOLLOWING
{
    const double EPSILON = 1e-9;
    bool IsDoubleEqualtoZero(double value, double mark = EPSILON);
    double getNumberFromXMLRPC(XmlRpc::XmlRpcValue &value, const std::string &full_param_name);
    std::vector<geometry_msgs::Point32> makeFootprintFromXMLRPC(XmlRpc::XmlRpcValue &footprint_xmlrpc, const std::string &full_param_name);
    bool IsKeyTarget(geometry_msgs::Point last_target_pos, geometry_msgs::Point cur_target_pos);
}

#endif // UTILS_H