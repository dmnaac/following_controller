#!/bin/bash

sleep 2

xfce4-terminal -T Master -e "bash -iC 'sleep 3; echo start!; $SHELL'" \
--tab -T roscore -e "bash -ic 'roscore; $SHELL'" \
--tab -T realsense -e "bash -ic 'sleep 5; roslaunch realsense2_camera rs_d435i_camera.launch; --wait; $SHELL'" \
--tab -T teleop -e "bash -c 'sleep 5; rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=cmd_vel_x; --wait; $SHELL'" &