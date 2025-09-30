#!/bin/bash
# Fail fast, but allow unbound vars while sourcing ROS env
set -Ee -o pipefail
export AMENT_TRACE_SETUP_FILES=0

log() { echo "[webapp] $*"; }

# --- Source ROS 2 and (optional) overlay without nounset issues --------------
# Option A: temporarily relax nounset just for sourcing
set +u
if [ -f /opt/ros/humble/setup.bash ]; then
  . /opt/ros/humble/setup.bash
fi
if [ -f /ros2_ws/install/setup.bash ]; then
  log "Using overlay at /ros2_ws"
  . /ros2_ws/install/setup.bash
else
  log "No /ros2_ws overlay detected. Continuing without it."
fi
set -u

# --- Start optional ROS-side helpers (don't kill container if missing) -------
( ros2 run tf2_web_republisher tf2_web_republisher \
  || log "tf2_web_republisher not found or failed to start" ) & sleep 2

( ros2 run web_video_server web_video_server --ros-args -p port:=11315 \
  || log "web_video_server not found or failed to start" ) & sleep 2

( ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
  || log "rosbridge_server launch failed" ) & sleep 2

# --- Nginx static site + proxies --------------------------------------------
rm -f /etc/nginx/sites-enabled/default || true
ln -sf /etc/nginx/sites-available/nginx.conf /etc/nginx/sites-enabled/nginx.conf || true
mkdir -p /run/nginx

log "Validating nginx config…"
nginx -t

log "Starting nginx (foreground)…"
exec nginx -g 'daemon off;'
