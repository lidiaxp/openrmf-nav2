#!/bin/bash

# Name of the tmux session
SESSION_NAME="turtlebot3_nav2"

# Start a new tmux session
tmux new-session -d -s $SESSION_NAME -n gazebo

echo "Starting the TurtleBot3 world in Gazebo..."
tmux send-keys -t $SESSION_NAME:0 "ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False" C-m

sleep 4

# Create a new window for static transform publishers
tmux new-window -t $SESSION_NAME -n transforms
echo "Publishing static transforms..."
tmux send-keys -t $SESSION_NAME:1 "ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom" C-m

# Create a new window for the kill command
tmux new-window -t $SESSION_NAME -n kill
echo "Adding a window with the command to end the tmux session..."
tmux send-keys -t $SESSION_NAME:2 "tmux kill-session -t $SESSION_NAME"
echo "Command 'tmux kill-session -t $SESSION_NAME' left in the history of the 'kill' window."

# Attach to the tmux session
echo "All processes started. Connecting to tmux..."
tmux attach-session -t $SESSION_NAME
