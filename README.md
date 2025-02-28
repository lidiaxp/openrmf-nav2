
# Open-RMF + Nav2

This docker was tested using ros2 humble version. Feel free to test other ros2 versions.

The objective of this docker is set up robots within open-rmf and control them using nav2.

# Installation
To build the docker:

```
bash build.sh
```

And, to execute:

```
bash run.sh
```

# How to use open-rmf

To set up nav2 with open rmf, it is possible to leverage the [demos](https://github.com/open-rmf/rmf_demos/?tab=readme-ov-file#Office-World).

A basic demo is:

```
ros2 launch rmf_demos_gz office.launch.xml
```

You can request the robot to deliver a can of coke from `pantry` to `hardware_2` through the following:
```
ros2 run rmf_demos_tasks dispatch_delivery -p pantry -ph coke_dispenser -d hardware_2 -dh coke_ingestor --use_sim_time
```

You can also request the robot to move back and forth between `coe` and `lounge` through the following:
```
ros2 run rmf_demos_tasks dispatch_patrol -p coe lounge -n 3 --use_sim_time
```

We have installed `tmux` within the docker, so you do not have to open several sessions to run these scripts, it is possible to run with `tmux`.

# How to use the fleet manager

For single robots:
```
bash scripts/run_tbt3_single.sh
```

After it, you also have to set the inital pose estimation on RVIZ. Then, it is possible to send the tasks (this command is also available on terminal 6):

```
ros2 run rmf_demos_tasks dispatch_patrol -p north_west north_east south_east south_west -n 2 -st 0
```

For multi robots:
```
bash scripts/run_tbt3_fleet.sh
```

You have to click `Startup` in both RVIZ to start the SLAM, and the set up the initial pose estimation. To send the commands to the robots, we have default commads in the terminal 6 (to the robot 1) and 7 (to the robot 2).

- For both cases (single and multi robots), it is missing trying emulating different computers between the robots.

# How to use nav2

Run `gazebo` in a terminal and wait until it opens. It can takes around 2 or 3 minutes. 

When gazebo opens, close it, and run:

```
chmod +x scripts/run_turtlebot3_nav2.sh
./scripts/run_turtlebot3_nav2.sh
```

# Missing Features

At this moment, the docker have installed open-rmf and nav2. However, the integration is not completed. It is missing 5 crucial steps:

- [] Set up custom (or even different) robots to open-rmf with the fleet manager;
- [] Set up custom (or even different) environments to open-rmf;
- [~] Integrate nav2 with open-rmf, [example](https://github.com/open-rmf/free_fleet/) (Missing simulate in different computers/networks/robots);
- [] Set up custom planners and controllers in nav2;
- [] Add features of stairs.

To integrate nav2 with open-rmf, it is not a pre-requisite, but would be nice to already have set up custom robots, as we need to inform the description of the robot for the nav2.

Also, to set up custom planners and controllers, it is necessary to create nav2 plugins, as described [here](https://docs.nav2.org/plugin_tutorials/docs/writing_new_nav2planner_plugin.html) and [here](https://docs.nav2.org/plugin_tutorials/docs/writing_new_nav2controller_plugin.html). An example of the entire param file can be found [here](https://github.com/ros-navigation/navigation2/blob/4e5d2dfc66cb75eb390d614d13e5b64efbf30284/nav2_bringup/params/nav2_params.yaml#L364).

As I could understand, after create the plugin, you have to:

- Modify the planner name in planner_server/ros_parameters;
- Modify the controller name in controller_server/ros_parameters;
- Modify the cmd_vel topic in collision_monitor/ros_parameters (considering we will use more than 1 robot, and consequently, need custom cmd_vel topics).

# Utils

## Several  terminals

To use multiple terminals, you can start `tmux` when opening the Docker container. This allows you to manage multiple terminal sessions within the same container.

Alternatively, on your computer, open a new terminal and run `bash new_terminal.sh`.
