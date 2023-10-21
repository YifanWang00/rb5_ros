#!/bin/bash

proj_dir="/root/rb5_ws"
cd $proj_dir
catkin_make
source devel/setup.bash

# Check if tmux session exists
tmux has-session -t calibration 2>/dev/null

# $? is a special variable that holds the exit status of the last command executed
if [ $? != 0 ]; then
  # Navigate to the specified directory
  cd /root/rb5_ws

  # Create a new tmux session named 'calibration' with mouse mode enabled
  tmux new-session -d -s calibration -x 208 -y 65
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
  tmux send-keys "rosrun rb5_control mpi_control_node.py" C-m

  # Create a new window split vertically from the original pane
  tmux split-window -v

  # Navigate to the bottom pane (assuming it's pane number 2)
  tmux select-pane -t 2

  # Run source command and roslaunch command in the bottom pane
  tmux send-keys "source devel/setup.bash" C-m
  tmux send-keys "sleep 2" C-m
  tmux send-keys "rosrun key_joy key_joy_node.py" C-m

  # Attach to the tmux session to interact with it
  tmux attach -t calibration
else
  echo "Tmux session 'calibration' already exists. Restarting..."
  tmux kill-ses -t calibration
  /root/script/setup_calibration.sh
fi

