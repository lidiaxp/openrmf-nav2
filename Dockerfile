FROM osrf/ros:humble-desktop

ENV ROS_DISTRO=humble
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    build-essential \
    git \
    curl \
    wget \
    vim \
    tmux 

RUN apt-get update && apt-get install -y \
    lsb-release \
    software-properties-common \
    python3-pip \
    clang \
    clang-tools \
    lldb \
    lld \
    libstdc++-12-dev 

RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-simple-commander \
    ros-${ROS_DISTRO}-turtlebot3-simulations \
    ros-dev-tools \
    ros-${ROS_DISTRO}-rmf-building-map-tools \
    ros-${ROS_DISTRO}-control-msgs \
    ros-${ROS_DISTRO}-backward-ros \
    ros-${ROS_DISTRO}-test-msgs \
    ros-${ROS_DISTRO}-realtime-tools \
    ros-${ROS_DISTRO}-nlohmann-json-schema-validator-vendor \
    ros-${ROS_DISTRO}-vision-msgs \
    ros-${ROS_DISTRO}-ament-cmake-catch2 \
    liburdfdom-dev \
    libwebsocketpp-dev \
    ros-${ROS_DISTRO}-rmf-building-sim-common \
    ros-${ROS_DISTRO}-rmf-building-sim-gz-plugins\
    ros-${ROS_DISTRO}-turtlebot3 \
    ros-${ROS_DISTRO}-turtlebot3-simulations \
    ros-${ROS_DISTRO}-fastrtps 


RUN pip3 install --no-cache-dir \
    colcon-common-extensions \
    empy \
    pyros-genmsg \
    jinja2 \
    fastapi \
    uvicorn \
    flask-socketio

WORKDIR /root/rmf_ws/src
RUN git clone https://github.com/open-rmf/rmf_demos.git -b 2.0.3 \
    && git clone https://github.com/open-rmf/rmf.git \
    && git clone https://github.com/open-rmf/rmf_ros2.git \
    && git clone https://github.com/open-rmf/rmf_internal_msgs.git \
    && git clone https://github.com/open-rmf/rmf_battery.git \
    && git clone https://github.com/open-rmf/rmf_visualization.git -b $ROS_DISTRO \
    && git clone https://github.com/open-rmf/rmf_visualization_msgs.git \
    && git clone https://github.com/open-rmf/rmf_api_msgs.git \
    && git clone https://github.com/open-rmf/pybind11_json_vendor.git \
    && git clone https://github.com/open-rmf/rmf_simulation.git -b $ROS_DISTRO \
    && git clone https://github.com/open-rmf/rmf_task.git \
    && git clone https://github.com/open-rmf/menge_vendor.git \
    && git clone https://github.com/open-rmf/rmf_traffic.git -b $ROS_DISTRO \
    && git clone https://github.com/ament/ament_cmake.git -b $ROS_DISTRO \
    && git clone https://github.com/open-rmf/rmf_utils.git \
    && git clone https://github.com/ros-planning/navigation2.git -b $ROS_DISTRO \
    && git clone https://github.com/ros-controls/ros2_control.git -b $ROS_DISTRO

    WORKDIR /root/rmf_ws
RUN . /opt/ros/$ROS_DISTRO/setup.sh \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -r -y

RUN . /opt/ros/$ROS_DISTRO/setup.sh \
    && export CXX=g++ \
    && export CC=gcc \
    && colcon build --symlink-install

ENV TURTLEBOT3_MODEL=waffle
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/dev/null
ENV FASTRTPS_SHM_PORT=0

RUN echo 'alias kill_gazebo_server=pkill -9 gzserver' >> ~/.bashrc
RUN echo 'alias kill_gazebo_client=pkill -9 gzclient' >> ~/.bashrc
RUN echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models' >> ~/.bashrc
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
RUN echo "source /root/rmf_ws/install/setup.bash" >> ~/.bashrc
SHELL ["/bin/bash", "-c"]

WORKDIR /root/rmf_ws

ENTRYPOINT ["/bin/bash", "-c"]
CMD ["source /root/rmf_ws/install/setup.bash && ros2 launch rmf_demos_gz office.launch.xml headless:=1"]

# ros2 topic pub /goal_pose geometry_msgs/PoseStamped \
# '{
#   "header": {
#     "frame_id": "map",
#     "stamp": {
#       "sec": 0,
#       "nanosec": 0
#     }
#   },
#   "pose": {
#     "position": {
#       "x": 0.0,
#       "y": 0.0,
#       "z": 0.0
#     },
#     "orientation": {
#       "x": 0.0,
#       "y": 0.0,
#       "z": 0.0,
#       "w": 0.0
#     }
#   }
# }'

# export FASTRTPS_DEFAULT_PROFILES_FILE=/dev/null
# export FASTRTPS_SHM_PORT=0
