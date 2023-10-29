#!/bin/bash

proj_dir="/root/rb5_ws"
cd $proj_dir

# Check if tmux session exists
tmux has-session -t hw2 2>/dev/null

# $? is a special variable that holds the exit status of the last command executed
if [ $? != 0 ]; then
  # Navigate to the specified directory
  cd $proj_dir
  catkin_make
  source devel/setup.bash

  # Create a new tmux session named 'hw2' with mouse mode enabled
  tmux new-session -d -s hw2 -x 208 -y 65
  tmux set -g mouse on

  # Run roscore command and sleep for 2 seconds
  tmux send-keys "roscore" C-m

  # Create a new window split horizontally
  tmux split-window -h

  # Navigate to the left pane (pane numbering starts from 0)
  tmux select-pane -t 1

  # Run source command and roslaunch command in the left pane
  tmux send-keys "source devel/setup.bash" C-m
  tmux send-keys "sleep 1" C-m
  tmux send-keys "roslaunch rb5_vision rb_camera_main_ocv.launch" C-m

  # Create a new window split vertically from the original pane
  tmux split-window -v

  # Navigate to the bottom pane (assuming it's pane number 2)
  tmux select-pane -t 2

  # Run source command and roslaunch command in the bottom pane
  tmux send-keys "source devel/setup.bash" C-m
  tmux send-keys "sleep 2" C-m
  tmux send-keys "roslaunch tf static_world_tf_broadcaster.launch" C-m

  tmux split-window -h
  tmux select-pane -t 3
  tmux send-keys "source devel/setup.bash" C-m
  tmux send-keys "sleep 3" C-m
  tmux send-keys "rosrun april_detection april_detection_node" C-m


  # Attach to the tmux session to interact with it
  tmux attach -t hw2
else
  echo "Tmux session 'hw2' already exists. Restarting..."
  tmux kill-ses -t hw2
  ./hw2_initialization.sh
fi

