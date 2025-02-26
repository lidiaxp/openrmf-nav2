#!/bin/bash

SESSION_NAME="tbt3_fleet_manager"

tmux new-session -d -s $SESSION_NAME -n gazebo
echo "Starting the TurtleBot3 world in Gazebo..."
tmux send-keys -t $SESSION_NAME:0 "ros2 launch nav2_bringup unique_multi_tb3_simulation_launch.py" C-m

sleep 4

tmux new-window -t $SESSION_NAME -n transforms
echo "Publishing static transforms..."
# tmux send-keys -t $SESSION_NAME:1.0 "ros2" C-m
# tmux split-window -h -t $SESSION_NAME:transforms   
# # tmux split-window -v -t $SESSION_NAME:transforms.0   
# # tmux split-window -v -t $SESSION_NAME:transforms.1   
tmux send-keys -t $SESSION_NAME:1.0 "ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 --frame-id map --child-frame-id odom" C-m
# tmux send-keys -t $SESSION_NAME:1.0 "ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom" C-m
# tmux send-keys -t $SESSION_NAME:1.1 "ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map base_link" C-m
# tmux send-keys -t $SESSION_NAME:1.2 "ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map robot1/map" C-m
# tmux send-keys -t $SESSION_NAME:1.3 "ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map robot2/map" C-m

tmux new-window -t $SESSION_NAME -n transforms_2
echo "Publishing static transforms for robot1..."
tmux send-keys -t $SESSION_NAME:transforms_2.0 "ros2" C-m
# tmux split-window -h -t $SESSION_NAME:transforms_2     
# # tmux split-window -v -t $SESSION_NAME:transforms_2.0   
# tmux send-keys -t $SESSION_NAME:transforms_2.0 "ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 robot1/map robot1/base_footprint" C-m
# # sleep 0.5
# tmux send-keys -t $SESSION_NAME:transforms_2.1 "ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 robot1/base_footprint base_footprint" C-m
# # sleep 0.5
# # tmux send-keys -t $SESSION_NAME:transforms_2.2 "ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_footprint robot1/map" C-m

tmux new-window -t $SESSION_NAME -n transforms_3
echo "Publishing static transforms for robot2..."
tmux send-keys -t $SESSION_NAME:transforms_3.0 "ros2" C-m
# # tmux split-window -v -t $SESSION_NAME:transforms_3.0  
# tmux split-window -h -t $SESSION_NAME:transforms_3     
# tmux send-keys -t $SESSION_NAME:transforms_3.0 "ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 robot2/map robot2/base_footprint" C-m
# # sleep 0.5
# tmux send-keys -t $SESSION_NAME:transforms_3.1 "ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 robot2/base_footprint base_footprint" C-m
# # sleep 0.5
# # tmux send-keys -t $SESSION_NAME:transforms_3.2 "ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_footprint robot2/map" C-m

tmux new-window -t $SESSION_NAME -n zenohd
echo "Starting Zenoh daemon..."
tmux send-keys -t $SESSION_NAME:4 "zenohd" C-m
sleep 2

tmux new-window -t $SESSION_NAME -n zenoh_bridge
echo "Starting Zenoh Bridge..."
tmux send-keys -t $SESSION_NAME:5 "cd ~/rmf_ws/src && ./zenoh-bridge-ros2dds -c ~/rmf_ws/src/free_fleet/free_fleet_examples/config/zenoh/nav2_unique_multi_tb3_zenoh_bridge_ros2dds_client_config.json5" C-m
sleep 2

tmux new-window -t $SESSION_NAME -n free_fleet_world
echo "Launching Free Fleet TurtleBot3 World RMF..."
tmux send-keys -t $SESSION_NAME:6 "ros2 launch free_fleet_examples turtlebot3_world_rmf_common.launch.xml" C-m
sleep 2

tmux new-window -t $SESSION_NAME -n fleet_adapter
echo "Launching Free Fleet Nav2 Unique Multi TB3 Simulation Fleet Adapter..."
tmux send-keys -t $SESSION_NAME:7 "ros2 launch free_fleet_examples nav2_unique_multi_tb3_simulation_fleet_adapter.launch.xml" C-m
sleep 2

tmux new-window -t $SESSION_NAME -n send_commands
echo "Sending commands..."
tmux send-keys -t $SESSION_NAME:8 "" C-m

# tmux new-window -t $SESSION_NAME -n kill
# echo "Adding a window with the command to end the tmux session..."
# tmux send-keys -t $SESSION_NAME:9 "tmux kill-session -t $SESSION_NAME" C-m
# echo "Command 'tmux kill-session -t $SESSION_NAME' left in the history of the 'kill' window."
# tmux kill-session -t tbt3_fleet_manager

echo "All processes started. Connecting to tmux..."
tmux attach-session -t $SESSION_NAME

# ros2 run rqt_tf_tree rqt_tf_tree
# export ROS_DOMAIN_ID=55

# # robot1 to run clockwise around the map
# ros2 run rmf_demos_tasks dispatch_patrol \
#   -p north_west north_east south_east south_west \
#   -n 3 \
#   -st 0 \
#   -F turtlebot3 \
#   -R robot1

# # robot2 to run anti-clockwise around the map
# ros2 run rmf_demos_tasks dispatch_patrol \
#   -p south_west south_east north_east north_west \
#   -n 3 \
#   -st 0 \
#   -F turtlebot3 \
#   -R robot2




# #!/bin/bash

# SESSION_NAME="tbt3_fleet_manager"

# tmux new-session -d -s $SESSION_NAME -n gazebo
# echo "Starting the TurtleBot3 world in Gazebo..."
# tmux send-keys -t $SESSION_NAME:0 "ros2 launch nav2_bringup unique_multi_tb3_simulation_launch.py" C-m

# sleep 4

# tmux new-window -t $SESSION_NAME -n zenohd
# echo "Starting Zenoh daemon..."
# tmux send-keys -t $SESSION_NAME:1 "zenohd" C-m
# sleep 2

# tmux new-window -t $SESSION_NAME -n zenoh_bridge
# echo "Starting Zenoh Bridge..."
# tmux send-keys -t $SESSION_NAME:2 "cd ~/rmf_ws/src && ./zenoh-bridge-ros2dds -c ~/rmf_ws/src/free_fleet/free_fleet_examples/config/zenoh/nav2_unique_multi_tb3_zenoh_bridge_ros2dds_client_config.json5" C-m
# sleep 2

# tmux new-window -t $SESSION_NAME -n free_fleet_world
# echo "Launching Free Fleet TurtleBot3 World RMF..."
# tmux send-keys -t $SESSION_NAME:3 "export ROS_DOMAIN_ID=55 && ros2 launch free_fleet_examples turtlebot3_world_rmf_common.launch.xml" C-m
# sleep 2

# tmux new-window -t $SESSION_NAME -n fleet_adapter
# echo "Launching Free Fleet Nav2 Unique Multi TB3 Simulation Fleet Adapter..."
# tmux send-keys -t $SESSION_NAME:4 "export ROS_DOMAIN_ID=55 && ros2 launch free_fleet_examples nav2_unique_multi_tb3_simulation_fleet_adapter.launch.xml" C-m
# sleep 2