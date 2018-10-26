#! /usr/bin/env bash

source /home/nvidia/argonaut_ws/src/runscripts/jetson/initial_setup.sh
roslaunch --wait realsense2_camera rs_rgbd.launch
