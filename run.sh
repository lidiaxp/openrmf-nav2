xhost +
docker run -it --rm \
    -e ROS_MASTER_URI=http://$(hostname -I | awk '{print $1}'):11311 \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e XAUTHORITY=$XAUTH \
    -v $XAUTH:$XAUTH \
    -v $(pwd)/scripts:/root/rmf_ws/scripts \
    -v $(pwd)/nav_scripts/nav2_sample_launch.py:/root/rmf_ws/src/navigation2/nav2_bringup/launch/nav2_sample_launch.py \
    --net=host \
    --privileged \
    nav2 \
    /bin/bash
