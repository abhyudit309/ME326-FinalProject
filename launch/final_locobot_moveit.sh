#!/bin/bash
#Run '$chmod +x ./final_locobot_moveit.sh' before first run
gnome-terminal -x roslaunch interbotix_xslocobot_moveit xslocobot_moveit.launch robot_model:=locobot_wx250s show_lidar:=true use_actual:=false use_camera:=true use_gazebo:=false dof:=6 use_moveit_rviz:=false