#!/bin/bash

# Name of the tmux session
SESSION_NAME="turtlebot3_nav2"

# Start a new tmux session
tmux new-session -d -s $SESSION_NAME -n gazebo

echo "Starting the TurtleBot3 world in Gazebo..."
tmux send-keys -t $SESSION_NAME:0 "ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py" C-m

# Wait for Gazebo to load
sleep 5

# Create a new window for the Nav2 stack
tmux new-window -t $SESSION_NAME -n nav2
echo "Starting the Nav2 stack..."
tmux send-keys -t $SESSION_NAME:1 "ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true" C-m

# Wait for Nav2 to load
sleep 5

# Create a new window for RViz
tmux new-window -t $SESSION_NAME -n rviz
echo "Opening RViz for visualization..."
tmux send-keys -t $SESSION_NAME:2 "ros2 launch nav2_bringup rviz_launch.py" C-m

# Wait for RViz to load
sleep 2

# Create a new window for static transform publishers
tmux new-window -t $SESSION_NAME -n transforms
echo "Publishing static transforms..."
tmux send-keys -t $SESSION_NAME:3 "ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map base_link" C-m
tmux split-window -t $SESSION_NAME:3
tmux send-keys -t $SESSION_NAME:3.1 "ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link" C-m
tmux split-window -t $SESSION_NAME:3
tmux send-keys -t $SESSION_NAME:3.2 "ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link base_scan" C-m
tmux split-window -t $SESSION_NAME:3
tmux send-keys -t $SESSION_NAME:3.3 "ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom" C-m
tmux split-window -t $SESSION_NAME:3
tmux send-keys -t $SESSION_NAME:3.4 "ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link base_footprint" C-m

# Create a new window for the kill command
tmux new-window -t $SESSION_NAME -n kill
echo "Adding a window with the command to end the tmux session..."
tmux send-keys -t $SESSION_NAME:4 "tmux kill-session -t $SESSION_NAME"
echo "Command 'tmux kill-session -t $SESSION_NAME' left in the history of the 'kill' window."

# Attach to the tmux session
echo "All processes started. Connecting to tmux..."
tmux attach-session -t $SESSION_NAME
