#! /usr/bin/env bash

source /home/nvidia/argonaut_ws/src/runscripts/jetson/initial_setup.sh
roslaunch --wait hls_lfcd_lds_driver hlds_laser.launch
