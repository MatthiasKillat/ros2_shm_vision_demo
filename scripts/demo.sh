#!/bin/bash

session="demo"
window="ros2 vision demo"

ros2_ws="~/ade-home/ros2_rolling/"
package="ros2_shm_vision_demo"
video="./video/apex_ai.mp4"
#video="./video/bunny_full_hd.mp4"

tmux new-session -d -s $session
tmux rename-window -t 0 $window

tmux split-window -h
tmux split-window -v
tmux split-window -v

tmux select-pane -t 0
tmux split-window -v
tmux split-window -v

tmux select-pane -t 0
tmux split-window -v
tmux select-pane -t 4
tmux split-window -v

tmux set -g pane-border-format "#{pane_index} #T"
tmux set pane-border-status top

tmux select-pane -t 0 -T "camera"
tmux select-pane -t 1 -T "display"
tmux select-pane -t 2 -T "network"
tmux select-pane -t 3 -T "CPU"

tmux select-pane -t 4 -T "background filter"
tmux select-pane -t 5 -T "edge detector"
tmux select-pane -t 6 -T "optical flow"
tmux select-pane -t 7 -T "object detector"

tmux setw synchronize-panes
tmux send-keys -t $session "cd $ros2_ws" C-m
tmux send-keys -t $session "source install/setup.bash" C-m
tmux send-keys -t $session "clear" C-m
tmux setw synchronize-panes off

tmux send-keys -t 2 "bmon --use-si" C-m
tmux send-keys -t 3 "sar 1" C-m

tmux send-keys -t 1 "ros2 run $package display" C-m

tmux send-keys -t 4 "ros2 run $package filter" C-m
tmux send-keys -t 5 "ros2 run $package edge_detector" C-m
tmux send-keys -t 6 "ros2 run $package optical_flow" C-m
tmux send-keys -t 7 "ros2 run $package object_detector" C-m

tmux send-keys -t 0 "ros2 run $package camera $video" C-m

tmux select-pane -t 0

tmux attach -t $session