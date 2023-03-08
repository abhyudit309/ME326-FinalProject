#!/bin/bash
#Run '$chmod +x ./final_locobot_control.sh' before first run
gnome-terminal -x roslaunch interbotix_xslocobot_control xslocobot_control.launch robot_model:=locobot_wx250s show_lidar:=true use_camera:=true use_rviz:=false align_depth:=true use_base:=true use_static_transform_pub:=true
