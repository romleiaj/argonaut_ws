#! /usr/bin/env bash

SESSION="sensors"
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

tmux new-session -d -s $SESSION -n "realsense"
tmux send-keys "source $DIR/02_launch_lds.sh" C-m

tmux new-window -t $SESSION:1 -n "lidar"
tmux send-keys "source $DIR/01_launch_realsense.sh" C-m
