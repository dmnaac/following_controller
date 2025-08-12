#!/bin/bash

sleep 2

xfce4-terminal -T Following_Controller -e "bash -iC 'sleep 3; echo start!; $SHELL'" \
--tab -T rviz -e "bash -ic 'sleep 5; cd ~/Workspace/following_controller_ws; source devel/setup.bash; roslaunch following_controller visualization_tracking.launch --wait; $SHEEL'" \
--tab -T carto -e "bash -ic 'sleep 5; cd ~/Workspace/following_controller_ws; source devel/setup.bash; roslaunch following_controller tmrobot_2d.launch --wait; $SHELL'" \
--tab -T move_base -e "bash -ic 'sleep 5; cd ~/Workspace/following_controller_ws; source devel/setup.bash; roslaunch following_controller local_move_base.launch --wait; $SHELL'" \
--tab -T follower -e "bash -ic 'sleep 5; cd ~/Workspace/following_controller_ws; source devel/setup.bash; roslaunch following_controller following_controller.launch --wait; $SHELL'" &