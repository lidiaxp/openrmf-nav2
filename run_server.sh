docker run \
  --network host \
  -it \
  -e ROS_DOMAIN_ID=0 \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  ghcr.io/open-rmf/rmf-web/api-server:latest
