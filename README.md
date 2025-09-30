# Checkpoint 22: Docker for Robotics

This repository contains Docker ROS 2 images and compose files to operate and extend the **Fastbot** robot. Running a containerized ROS 2 environment enables testing in the **Gazebo** simulator and seamless deployment of **SLAM** and navigation on the real robot—without polluting your host setup.

> **You only need to clone this repo and run `docker-compose`**. Compose will fetch the needed images automatically.

---

## Prerequisites

- Docker Engine and Docker Compose

### Install on Ubuntu
```bash
sudo apt-get update
sudo apt-get install -y docker.io docker-compose
sudo systemctl enable --now docker
```

Optionally, enable Docker commands without `sudo`:
```bash
sudo usermod -aG docker $USER
newgrp docker
```

> **Compose v2 users:** If `docker-compose` isn’t available, use `docker compose` (space instead of hyphen).

---

## Getting Started

Clone this repository anywhere in your workspace:
```bash
git clone <this-repo-url>
cd fastbot_ros2_docker
```

---

## Simulation Containers

Running compose will download the required images and start three ROS 2 containers: **Gazebo** (robot + world), **SLAM**, and a **WebApp** (rosbridge + simple UI).

```bash
cd simulation
docker-compose up
```

Check running containers in a second terminal:
```bash
docker ps
```

**Expected result (example):**
```
CONTAINER ID   IMAGE                                        COMMAND                  CREATED          STATUS          PORTS                                                                 NAMES
a123d0b83cf3   mathros/mathiasrosas-cp22:fastbot-ros2-slam  "/bin/bash -c 'sleep…"   14 minutes ago   Up 14 minutes                                                                         fastbot-ros2-slam
397bd210f7e4   mathros/mathiasrosas-cp22:fastbot-ros2-webapp"/bin/bash /entrypoi…"   14 minutes ago   Up 14 minutes   0.0.0.0:9090->9090/tcp, :::9090->9090/tcp, 0.0.0.0:7000->80/tcp, :::7000->80/tcp   fastbot-ros2-webapp
5406243bd2d0   mathros/mathiasrosas-cp22:fastbot-ros2-gazebo"/bin/bash -c 'sourc…"   14 minutes ago   Up 14 minutes                                                                         fastbot-ros2-gazebo
```

### WebApp interaction

In The Construct environment, get the public URLs:
```bash
webpage_address      # returns the public URL of the web UI
rosbridge_address    # returns ws:// or wss:// URL for rosbridge
```

1. Open the **webpage URL** in your browser.
2. Paste the **rosbridge URL** into the *ROSBridge address* field and click **Connect**.
3. You should see odometry and the map; use the on-page joystick to drive the robot in simulation.

### Navigation (RViz2)

- Set an **initial pose** in RViz using **2D Pose Estimate** to help localization.
- Send navigation goals using **2D Nav Goal** once localization activates.
- Alternatively, teleop from a terminal:
  ```bash
  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=fastbot/cmd_vel
  ```

To stop the simulation:
```bash
# In the compose terminal, press Ctrl+C
# Or from repo/simulation
docker-compose down
```

---

## Real Robot Containers

Bringup for the real robot (camera, LiDAR, motors) and on-device SLAM:

```bash
cd real
docker-compose up
```

Verify topics (from a shell with ROS 2 sourced):
```bash
ros2 topic list
# Expect to see (examples):
# /scan
# /camera/image_raw
# /fastbot/cmd_vel
```

> If you hit serial/USB permission issues for `/dev/ttyUSB*`, ensure devices are mapped in `docker-compose.yml` (or use `privileged: true`) and your host user has the right group memberships (e.g., `dialout`).

To stop:
```bash
docker-compose down
```

---

## Tips & Troubleshooting

- **Compose v1 vs v2**: use `docker-compose` or `docker compose` depending on what’s installed.
- **Container won’t start / exits**:
  ```bash
  docker-compose logs <service-name>
  ```
- **Gazebo/RViz hiccups**: stop and relaunch; if a stale `gzserver` remains,
  ```bash
  ps aux | grep gz
  kill -9 <pid>
  ```
- **Autostart on reboot (optional)**: set `restart: always` in compose and ensure Docker auto-starts:
  ```bash
  sudo systemctl enable docker
  ```

---

## TL;DR
```bash
# 1) Install Docker + Compose
# 2) Clone the repo
git clone <this-repo-url>
cd fastbot_ros2_docker

# 3) Simulation
cd simulation && docker-compose up

# 4) Real robot (on the robot)
cd ../real && docker-compose up
```
