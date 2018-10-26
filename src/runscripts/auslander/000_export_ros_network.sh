#! /usr/bin/env bash

# NOTE: Must call "source" on this script to set variables

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
source ${DIR}/../../../devel/setup.bash
export ROS_MASTER_URI="http://192.168.119.20:11311/"
echo $ROS_MASTER_URI
export ROS_IP="192.168.119.15"
echo $ROS_IP
