# Checkpoint 22 — Docker for Robotics (FastBot)
## **fastbot_ros2_docker** — ROS 2 Humble containers for **simulation** and **real robot**

This repository contains everything you need to containerize and run the **FastBot** robot with **ROS 2 Humble**, both in **Gazebo simulation** and on the **real robot**.  
It is designed to match the grading criteria for **Checkpoint 22 — Docker for Robotics**.

> **Note on ROS versions**  
> Even if your host uses another ROS version (e.g., Galactic), these Docker images are based on **ROS 2 Humble**, which is the distribution supported by FastBot. Docker isolates the runtime so the host ROS version does not matter.

---

## Contents

- [Repository structure](#-repository-structure)
- [Prerequisites](#-prerequisites)
- [Download this repository](#-download-this-repository)
- [Use prebuilt images (recommended)](#-use-prebuilt-images-recommended)
- [Task 1 — Simulation](#-task1--simulation)
  - [Start the stack](#start-the-stack)
  - [Verify containers and images](#verify-containers-and-images)
  - [Drive the robot & run mapping](#drive-the-robot--run-mapping)
  - [Website URLs (webapp)](#website-urls-webapp)
  - [Stop the stack](#stop-the-stack)
- [Task 2 — Real robot](#-task2--real-robot)
  - [Start on the robot](#start-on-the-robot)
  - [Verify topics](#verify-topics)
  - [Autostart on reboot](#autostart-on-reboot)
  - [Access from an external computer](#access-from-an-external-computer)
- [Build locally (optional)](#-build-locally-optional)
- [Push to Docker Hub (optional)](#-push-to-docker-hub-optional)
- [Troubleshooting](#-troubleshooting)
- [Grader checklist (what to test)](#-grader-checklist-what-to-test)
- [TL;DR](#tldr)

---

## 📂 Repository structure

```
fastbot_ros2_docker/
├── simulation/              # Task 1: Simulation Dockerfiles & compose
│   ├── Dockerfile.gazebo    # Gazebo + ROS 2 simulation
│   ├── Dockerfile.slam      # Mapping (Cartographer)
│   ├── Dockerfile.webapp    # Web application (rosbridge + web-video)
│   └── docker-compose.yml   # starts gazebo, slam, webapp
└── real/                    # Task 2: Real-robot Dockerfiles & compose
    ├── Dockerfile.real      # Bringup: camera, LiDAR, motors
    ├── Dockerfile.slam      # Real-robot SLAM (Cartographer)
    └── docker-compose.yml   # starts real and slam-real containers
```

---

## ✅ Prerequisites

- **Docker** and **Docker Compose**:
  ```bash
  sudo apt-get update
  sudo apt-get install -y docker.io docker-compose
  sudo systemctl enable --now docker

  # (optional, recommended) use Docker without sudo
  sudo usermod -aG docker $USER
  newgrp docker
  ```

- **Git** configured:
  ```bash
  git config --global user.name  "Your Name"
  git config --global user.email "you@example.com"
  ```

> **Compose v2 users:** If `docker-compose` is not available, replace commands with `docker compose` (space instead of hyphen).

---

## ⬇️ Download this repository

```bash
cd ~/ros2_ws/src
git clone https://github.com/mathrosas/fastbot_ros2_docker.git
cd fastbot_ros2_docker
```

---

## 📦 Use prebuilt images (recommended)

All required images are published to Docker Hub under **mathros/mathiasrosas-cp22**:  
**https://hub.docker.com/repository/docker/mathros/mathiasrosas-cp22/general**

**Available tags (5):**
- `fastbot-ros2-gazebo`
- `fastbot-ros2-slam`
- `fastbot-ros2-webapp`
- `fastbot-ros2-real`
- `fastbot-ros2-slam-real`

Pull them once (or let `docker-compose` pull automatically):
```bash
docker pull mathros/mathiasrosas-cp22:fastbot-ros2-gazebo
docker pull mathros/mathiasrosas-cp22:fastbot-ros2-slam
docker pull mathros/mathiasrosas-cp22:fastbot-ros2-webapp
docker pull mathros/mathiasrosas-cp22:fastbot-ros2-real
docker pull mathros/mathiasrosas-cp22:fastbot-ros2-slam-real
```

> The `docker-compose.yml` files reference these tags so you don’t need to build locally unless you want to modify the images.

---

## 🎮 Task 1 — Simulation

All simulation containers run **ROS 2 Humble** and include Gazebo, Cartographer mapping, and a basic web interface.

### Start the stack
```bash
cd simulation
docker-compose up -d
```

### Verify containers and images
Check that the three containers are up:
```bash
docker ps | grep fastbot-ros2-
# fastbot-ros2-gazebo
# fastbot-ros2-slam
# fastbot-ros2-webapp
```

If the grader lists images:
```bash
docker images | grep mathiasrosas-cp22
# mathros/mathiasrosas-cp22   fastbot-ros2-gazebo
# mathros/mathiasrosas-cp22   fastbot-ros2-slam
# mathros/mathiasrosas-cp22   fastbot-ros2-webapp
```

### Drive the robot & run mapping
Open a terminal on the host (or inside a tools container) and run:
```bash
source ~/ros2_ws/install/setup.bash

# Teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=fastbot/cmd_vel

# Mapping (Cartographer)
ros2 launch fastbot_slam cartographer.launch.py
```

### Website URLs (webapp)
In The Construct environment use the helper commands:
```bash
webpage_address      # returns the public URL of the web UI
rosbridge_address    # returns ws:// URL for rosbridge
```
From the website you should be able to:
- Connect via rosbridge
- Drive the robot with the on-page joystick
- Visualize the map being generated

### Stop the stack
```bash
docker-compose down
```

---

## 🤖 Task 2 — Real robot

These containers bring up the **real** FastBot hardware (camera, LiDAR, motors) and SLAM on the robot’s computer.

### Start on the robot
```bash
cd real
docker-compose up -d
```

Confirm containers:
```bash
docker ps | grep fastbot-ros2-
# fastbot-ros2-real
# fastbot-ros2-slam-real
```

### Verify topics
```bash
ros2 topic list
# Expect to see topics like:
# /scan
# /camera/image_raw
# /fastbot/cmd_vel
```

> If you see permission errors for `/dev/ttyUSB*`, ensure your `docker-compose.yml` maps the devices correctly (or uses `privileged: true`).

### Autostart on reboot
Compose files are set with `restart: always`. Ensure the Docker daemon starts on boot:
```bash
sudo systemctl enable docker
```
Now power-cycle the robot; the containers should start automatically.

### Access from an external computer
- Source your ROS 2 environment and ensure network visibility (same LAN).
- You should be able to:
  - List the robot topics: `ros2 topic list`
  - Visualize the map in RViz2 (subscribe to `/map` and relevant TF).

---

## 🛠 Build locally (optional)

If you wish to modify or rebuild images instead of using the prebuilt ones:
```bash
# Simulation images
cd simulation
docker build -f Dockerfile.gazebo -t fastbot-ros2-gazebo .
docker build -f Dockerfile.slam   -t fastbot-ros2-slam   .
docker build -f Dockerfile.webapp -t fastbot-ros2-webapp .

# Real-robot images
cd ../real
docker build -f Dockerfile.real -t fastbot-ros2-real .
docker build -f Dockerfile.slam -t fastbot-ros2-slam-real .
```

> If you want Compose to use your locally built images instead of the Hub tags, either retag them as
> `mathros/mathiasrosas-cp22:<tag>` or edit the `image:` fields in the compose files.

---

## 📤 Push to Docker Hub (optional)

Tag and push (example for the gazebo image; repeat for others):
```bash
# Replace with your Docker Hub namespace if you’re pushing to a different registry
docker tag fastbot-ros2-gazebo mathros/mathiasrosas-cp22:fastbot-ros2-gazebo
docker push mathros/mathiasrosas-cp22:fastbot-ros2-gazebo
```

---

## 📝 Troubleshooting

- **Simulation/Gazebo didn’t load first try**  
  Stop and relaunch the simulation; it usually works the second time.

- **Stale Gazebo server (`gzserver`) after stopping**  
  ```bash
  ps faux | grep gz
  kill -9 <process_id>
  ```

- **Container won’t start / crashes immediately**  
  ```bash
  docker-compose logs <service-name>
  ```

- **USB/serial permissions on the robot**  
  Map devices in compose or use `privileged: true`. On the host, you can also add your user to groups like `dialout` (then re-login).

- **Compose v2 vs v1**  
  If `docker-compose` is not found, use `docker compose` instead.

---

## ✅ Grader checklist (what to test)

**Task 1 — Simulation**
1. Go to `~/ros2_ws/src/fastbot_ros2_docker/simulation`  
   ```bash
   docker-compose up
   docker ps
   ```
   Expect **three containers** running:
   - `fastbot-ros2-gazebo`
   - `fastbot-ros2-slam`
   - `fastbot-ros2-webapp`

2. Verify images are present:
   ```bash
   docker images | grep mathiasrosas-cp22
   ```

3. Gazebo is launched (visible in Graphical Tools).

4. Website works (use `webpage_address`) and connects via rosbridge (use `rosbridge_address`):
   - Connect from the website to robot systems
   - Drive the robot with joystick
   - Visualize the map on the website

**Task 2 — Real robot**
1. On the robot:  
   ```bash
   cd ~/ros2_ws/src/fastbot_ros2_docker/real
   docker-compose up
   docker ps
   ```
   Expect **two containers** running:
   - `fastbot-ros2-real`
   - `fastbot-ros2-slam-real`

2. Verify topics:
   ```bash
   ros2 topic list
   ```
   - Laser topic exists ✔️
   - Camera topic exists ✔️
   - Velocity command topic exists ✔️

3. Power-cycle robot; containers should **start automatically** (`restart: always`).

4. From an external computer:
   - You can list robot topics ✔️
   - You can visualize the map in RViz2 ✔️

---

## TL;DR

```bash
# 0) Install Docker + Compose (and add your user to the docker group)
# 1) Get the repo
cd ~/ros2_ws/src
git clone https://github.com/mathrosas/fastbot_ros2_docker.git
cd fastbot_ros2_docker

# 2) Pull prebuilt images (one-time)
docker pull mathros/mathiasrosas-cp22:fastbot-ros2-gazebo
docker pull mathros/mathiasrosas-cp22:fastbot-ros2-slam
docker pull mathros/mathiasrosas-cp22:fastbot-ros2-webapp
docker pull mathros/mathiasrosas-cp22:fastbot-ros2-real
docker pull mathros/mathiasrosas-cp22:fastbot-ros2-slam-real

# 3a) Simulation
cd simulation
docker-compose up -d
docker ps | grep fastbot-ros2-

# 3b) Real robot
cd ../real
docker-compose up -d
docker ps | grep fastbot-ros2-
```

Happy robotics! 🚀
