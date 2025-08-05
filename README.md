**fastbot_ros2_docker**

This repository contains everything you need to containerize and run the FastBot robot in both simulation and on real hardware, using ROS 2 Humble.

---

## 📂 Repository Structure

```
fastbot_ros2_docker/
├── simulation/              # Task 1: Simulation Dockerfiles & compose
│   ├── Dockerfile.gazebo    # Gazebo + ROS2 simulation
│   ├── Dockerfile.slam      # Mapping (Cartographer)
│   ├── Dockerfile.webapp    # Web application (rosbridge + web-video)
│   └── docker-compose.yml   # starts gazebo, slam, webapp
└── real/                    # Task 2: Real-robot Dockerfiles & compose
    ├── Dockerfile.real      # Bringup: camera, LiDAR, motors
    ├── Dockerfile.slam      # Real-robot SLAM (Cartographer)
    └── docker-compose.yml   # starts real and slam-real containers
```

---

## 🚀 Getting Started

### 1. Clone this repository

```bash
cd ~/ros2_ws/src
git clone https://github.com/mathrosas/fastbot_ros2_docker.git
cd fastbot_ros2_docker
```

### 2. Prerequisites

* **Docker & Docker Compose** installed:

  ```bash
  sudo apt-get update
  sudo apt-get install -y docker.io docker-compose
  sudo systemctl enable --now docker
  ```
* **Git** configured:

  ```bash
  git config --global user.name "Your Name"
  git config --global user.email "you@example.com"
  ```
* **Docker Hub account** for pushing images (optional).

---

## 🎮 Task 1: Simulation

All simulation containers are based on **ROS 2 Humble** and include Gazebo, mapping, and web interface.

1. Enter the `simulation` folder:

   ```bash
   cd simulation
   ```

2. Build and start containers:

   ```bash
   docker-compose up -d
   ```

3. Verify that the three containers are running:

   ```bash
   docker ps | grep fastbot-ros2-
   # fastbot-ros2-gazebo
   # fastbot-ros2-slam
   # fastbot-ros2-webapp
   ```

4. Interact with the simulation:

   * **Gazebo GUI** opens automatically inside the `fastbot-ros2-gazebo` container.
   * **Teleoperate**:

     ```bash
     source ~/ros2_ws/install/setup.bash
     ros2 run teleop_twist_keyboard teleop_twist_keyboard \
       --ros-args --remap cmd_vel:=fastbot/cmd_vel
     ```
   * **Mapping**:

     ```bash
     source ~/ros2_ws/install/setup.bash
     ros2 launch fastbot_slam cartographer.launch.py
     ```

5. **Stop** the simulation:

   ```bash
   docker-compose down
   ```

---

## 🤖 Task 2: Real Robot

These containers bring up the real FastBot hardware (camera, LiDAR, motors) and SLAM on the physical robot.

1. Enter the `real` folder:

   ```bash
   cd ../real
   ```
2. Build and launch:

   ```bash
   docker-compose up -d
   ```
3. Check running containers:

   ```bash
   docker ps | grep fastbot-ros2-
   # fastbot-ros2-real
   # fastbot-ros2-slam-real
   ```
4. Confirm ROS2 topics on the robot:

   ```bash
   ros2 topic list
   # Expect /scan, /camera/image_raw, /fastbot/cmd_vel, etc.
   ```
5. **Auto-start on reboot** is enabled via `restart: always` in the compose file. Ensure Docker daemon is enabled:

   ```bash
   sudo systemctl enable docker
   ```
6. **Stop** real-robot containers:

   ```bash
   docker-compose down
   ```

---

## 📤 Pushing to Docker Hub (optional)

After verifying locally, tag and push each image:

```bash
# replace <your_dockerhub> with your Docker Hub username
docker tag fastbot-ros2-gazebo    <your_dockerhub>/cp22:fastbot-ros2-gazebo
docker push <your_dockerhub>/cp22:fastbot-ros2-gazebo
# repeat for fastbot-ros2-slam, fastbot-ros2-webapp,
# fastbot-ros2-real, fastbot-ros2-slam-real
```

---

## 📝 Troubleshooting

* **Container won’t start?**

  ```bash
  ```

docker-compose logs <service-name>

````
- **Stale Gazebo server?**  
```bash
ps faux | grep gz
kill -9 <pid>
````

* **Permission denied** on `/dev/ttyUSB*`: ensure `privileged: true` or proper `devices:` mapping.

---

Happy robotics! 🚀
