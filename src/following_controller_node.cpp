#include "following_controller/follower.h"
int main(int argc, char **argv)
{
    ros::init(argc, argv, "following_controller_node");
    ros::NodeHandle nh;
    FOLLOWING::Follower follower(nh);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::waitForShutdown();
    spinner.stop();
    return 0;
}