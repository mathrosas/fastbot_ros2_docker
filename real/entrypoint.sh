#!/bin/bash
set -e
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=/var/lib/theconstruct.rrl/cyclonedds.xml
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/colcon_ws/install/setup.bash"
exec bash -c "$@"