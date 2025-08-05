#!/bin/bash
set -eo pipefail
export AMENT_TRACE_SETUP_FILES=0

# 1) source ROS 2 & your overlay
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

ros2 run tf2_web_republisher_py tf2_web_republisher &

sleep 5

ros2 run web_video_server web_video_server --ros-args -p port:=11315 &

sleep 5

ros2 launch rosbridge_server rosbridge_websocket_launch.xml &

sleep 5

cd /ros2_ws/src/fastbot_webapp
exec python3 -m http.server 7000
