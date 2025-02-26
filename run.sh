xhost +
docker run -it --rm \
    -e ROS_MASTER_URI=http://$(hostname -I | awk '{print $1}'):11311 \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e XAUTHORITY=$XAUTH \
    -v $XAUTH:$XAUTH \
    -v $(pwd)/scripts:/root/rmf_ws/scripts \
    -v $(pwd)/scripts/tb3_sandbox.yaml:/opt/ros/jazzy/share/nav2_bringup/maps/tb3_sandbox.yaml \
    --net=host \
    --privileged \
    nav2 \
    /bin/bash
