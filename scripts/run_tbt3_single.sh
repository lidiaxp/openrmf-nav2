#!/bin/bash

SESSION_NAME="tbt3_single_fleet_manager"

tmux new-session -d -s $SESSION_NAME -n gazebo
echo "Starting the TurtleBot3 world in Gazebo..."
tmux send-keys -t $SESSION_NAME:0 "ros2 launch nav2_bringup tb3_simulation_launch.py headless:=0" C-m

sleep 4

tmux new-window -t $SESSION_NAME -n transforms
echo "Publishing static transforms..."
tmux split-window -h -t $SESSION_NAME:transforms   
tmux send-keys -t $SESSION_NAME:1.0 "ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom" C-m
tmux send-keys -t $SESSION_NAME:1.1 "ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map base_footprint" C-m

tmux new-window -t $SESSION_NAME -n zenohd
echo "Starting Zenoh daemon..."
tmux send-keys -t $SESSION_NAME:2 "zenohd" C-m
sleep 1

tmux new-window -t $SESSION_NAME -n zenoh_bridge
echo "Starting Zenoh Bridge..."
tmux send-keys -t $SESSION_NAME:3 "cd ~/rmf_ws/src && ./zenoh-bridge-ros2dds -c ~/rmf_ws/src/free_fleet/free_fleet_examples/config/zenoh/nav2_tb3_zenoh_bridge_ros2dds_client_config.json5" C-m
sleep 1

tmux new-window -t $SESSION_NAME -n free_fleet_world
echo "Launching Free Fleet TurtleBot3 World RMF..."
tmux send-keys -t $SESSION_NAME:4 "ros2 launch free_fleet_examples turtlebot3_world_rmf_common.launch.xml" C-m
sleep 1

tmux new-window -t $SESSION_NAME -n fleet_adapter
echo "Launching Free Fleet Nav2 Unique Multi TB3 Simulation Fleet Adapter..."
tmux send-keys -t $SESSION_NAME:5 "ros2 launch free_fleet_examples nav2_tb3_simulation_fleet_adapter.launch.xml" C-m
sleep 1

tmux new-window -t $SESSION_NAME -n send_commands
echo "Sending commands..."
tmux send-keys -t $SESSION_NAME:6 "ros2 run rmf_demos_tasks dispatch_patrol -p north_west north_east south_east south_west -n 2 -st 0" " "
sleep 0.5

tmux new-window -t $SESSION_NAME -n kill
echo "Adding a window with the command to end the tmux session..."
tmux send-keys -t $SESSION_NAME:7 "tmux kill-session -t tbt3_single_fleet_manager & pkill -9 gzserver & pkill -9 gzclient" " "
echo "Command is written in the 'kill' window but will require manual execution."

echo "All processes started. Connecting to tmux..."
tmux attach-session -t $SESSION_NAME

# ros2 run rqt_tf_tree rqt_tf_tree
# export ROS_DOMAIN_ID=55
