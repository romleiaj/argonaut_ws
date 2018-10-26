#! /usr/bin/env bash

#NOTE: Must call "source" to set variables

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
source ${DIR}/../../../devel/setup.bash
export ROS_MASTER_URI="http://192.168.119.20:11311/"
export ROS_IP="192.168.119.20"
roslaunch --wait realsense2_camera rs_rgbd.launch
