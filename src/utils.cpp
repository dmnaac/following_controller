#include "following_controller/utils.h"

namespace FOLLOWING
{
    bool IsDoubleEqualtoZero(double value, double mark)
    {
        return std::fabs(value) < mark;
    }

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

    std::vector<geometry_msgs::Point32> makeFootprintFromXMLRPC(XmlRpc::XmlRpcValue &footprint_xmlrpc, const std::string &full_param_name)
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

    bool IsKeyTarget(geometry_msgs::Point last_target_pos, geometry_msgs::Point cur_target_pos)
    {
        double dx = cur_target_pos.x - last_target_pos.x;
        double dy = cur_target_pos.y - last_target_pos.y;
        double dist = std::sqrt(dx * dx + dy * dy);
        if (dist > 0.5)
        {
            return true;
        }
        return false;
    }

    double rad2deg(double rad)
    {
        return rad * 180.0 / M_PI;
    }

    double deg2rad(double deg)
    {
        return deg * M_PI / 180.0;
    }

    double NormalizeAngle(double angle)
    {
        angle = fmod(angle, 2 * M_PI);

        if (angle > M_PI)
        {
            angle -= 2 * M_PI;
        }
        else if (angle < -M_PI)
        {
            angle += 2 * M_PI;
        }

        return angle;
    }
}