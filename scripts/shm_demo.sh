#!/bin/bash

session="demo"
window="ros2 vision demo"

ros2_ws="$HOME/code/ros/rolling"
package="shm_vision_demo"
iceoryx_path="$ros2_ws/install/iceoryx_posh/bin"
pkg_share_path="${ros2_ws}/install/$package/share/$package"

# includ helper functions
source ./utils.sh

video=$(getVideoPath $1)
checkDeps tmux

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
tmux select-pane -t 2 -T "roudi"
tmux select-pane -t 3 -T "introspection"

tmux select-pane -t 4 -T "background filter"
tmux select-pane -t 5 -T "edge detector"
tmux select-pane -t 6 -T "optical flow"
tmux select-pane -t 7 -T "object detector"

tmux setw synchronize-panes
tmux send-keys -t $session "cd $ros2_ws" C-m
tmux send-keys -t $session "source install/setup.bash" C-m
tmux send-keys -t $session "export CYCLONEDDS_URI=file://$pkg_share_path/config/cyclonedds.xml" C-m
tmux send-keys -t $session "clear" C-m
tmux setw synchronize-panes off

tmux send-keys -t 3 "$iceoryx_path/iox-introspection-client --mempool" C-m
tmux send-keys -t 2 "$iceoryx_path/iox-roudi -c $pkg_share_path/config/roudi_config.toml" C-m

tmux send-keys -t 1 "ros2 run $package display" C-m

tmux send-keys -t 4 "ros2 run $package filter" C-m
tmux send-keys -t 5 "ros2 run $package edge_detector" C-m
tmux send-keys -t 6 "ros2 run $package optical_flow" C-m
tmux send-keys -t 7 "ros2 run $package object_detector" C-m

tmux send-keys -t 0 "ros2 run $package camera $video" C-m

tmux select-pane -t 3

tmux attach -t $session
