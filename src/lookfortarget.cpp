#include "following_controller/lookfortarget.h"

LookforTargetServer::LookforTargetServer(ros::NodeHandle &nh, const std::string &action_name) : nh_(nh), as_(nh, action_name, boost::bind(&LookforTargetActionServer::ExecuteCB, this, _1), false), action_name_(action_name)
{
    cmdVelPub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel_x", 10);

    as_.start();
    ROS_INFO("Action server starts: %s", action_name.c_str());
}

LookforTargetServer::~LookforTargetServer()
{
}

LookforTargetClient::LookforTargetClient(/* args */)
{
}

LookforTargetClient::~LookforTargetClient()
{
}