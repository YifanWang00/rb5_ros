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
  tmux select-pane -t 1
  tmux send-keys "source devel/setup.bash" C-m
  tmux send-keys "rosrun tf_broadcaster tf_broadcaster_node" C-m

  # Create a new window split vertically from the original pane
  tmux split-window -v
  tmux select-pane -t 2
  tmux send-keys "source devel/setup.bash" C-m
  tmux send-keys "sleep 3" C-m
  tmux send-keys "/root/rb5_ws/src/rb5_ros/map_to_markers_tf.sh" C-m

  tmux split-window -v
  tmux select-pane -t 3
  tmux send-keys "source devel/setup.bash" C-m
  tmux send-keys "sleep 3" C-m
  tmux send-keys "rosrun april_detection april_detection_node" C-m

  tmux split-window -v
  tmux select-pane -t 4
  tmux send-keys "source devel/setup.bash" C-m
  tmux send-keys "sleep 3" C-m
  tmux send-keys "roslaunch rb5_vision rb_camera_main_ocv.launch" C-m

  tmux select-pane -t 0
  tmux split-window -v
  tmux select-pane -t 5
  tmux send-keys "source devel/setup.bash" C-m
  tmux send-keys "rviz" C-m


  # Attach to the tmux session to interact with it
  tmux attach -t hw2
else
  echo "Tmux session 'hw2' already exists. Restarting..."
  tmux kill-ses -t hw2
  /root/rb5_ws/src/rb5_ros/aprilTag_test.sh
fi
