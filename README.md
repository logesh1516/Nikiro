# 🤖 Nikiro AMR

Nikiro is a ROS 2-based **Autonomous Mobile Robot (AMR)** designed for indoor navigation using a differential drive system. It leverages **Gazebo, RViz, Nav2, and SLAM Toolbox** for simulation, mapping, localization, and path planning.

> 📹 [Watch full demo on LinkedIn]([https://www.linkedin.com/posts/activity-7328418940064280576](https://www.linkedin.com/posts/logesh-s-17674824b_ros2-amr-mycobot-ugcPost-7328418940064280576-ZZe8?utm_source=share&utm_medium=member_desktop&rcm=ACoAAD3ePuoBQMq-eymcajH24mmiFO1cDOwpDD0)
> Based on [linorobot2](https://github.com/linorobot/linorobot2)

---

## 📦 Related Repositories

| Repository | Description |
|------------|-------------|
| [Nikiro_simulation](https://github.com/logesh1516/Nikiro_simulation) | Gazebo simulation environment |
| [Nikiro_hardware](https://github.com/logesh1516/Nikiro_hardware) | Hardware bringup & microcontroller interface |
| [Nikiro_docker](https://github.com/logesh1516/Nikiro_docker) | Docker setup for Nikiro |

---

## 🛠️ Tech Stack

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)
![Python](https://img.shields.io/badge/Python-3.10-blue)
![C++](https://img.shields.io/badge/C++-17-blue)
![Gazebo](https://img.shields.io/badge/Gazebo-Fortress-orange)
![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-E95420)

---

## 🏗️ System Architecture

- **SLAM**: SLAM Toolbox for map generation
- **Localization**: AMCL via Nav2
- **Navigation**: Nav2 stack with path planning
- **LiDAR**: YDLidar X2 (`ydlidar_ros2_driver`)
- **Drive**: Differential drive with micro-ROS serial interface
- **URDF**: Custom Xacro model with differential drive plugin

---

## 📸 Robot Model

![Nikiro URDF](https://github.com/user-attachments/assets/1fb7a25f-acb4-4d96-beb2-737281d3180f)

---

## ⚙️ Installation

```bash
mkdir -p nikiro_amr/src
cd nikiro_amr/src
git clone https://github.com/logesh1516/Nikiro.git
cd ..
colcon build && source install/setup.bash
```

### Environment Setup

```bash
echo "export NIKIRO_LASER_SENSOR=ydlidar" >> ~/.bashrc
source ~/.bashrc
```

---

## 🚀 Running the Robot

### Real Robot

```bash
ros2 launch nikiro_amr_bringup bringup.launch.py
```

Optional parameters:
- `base_serial_port` — Serial port of microcontroller (default: `/dev/ttyACM0`)
- `micro_ros_baudrate` — micro-ROS baudrate (default: `115200`)

```bash
ros2 launch nikiro_amr_bringup bringup.launch.py \
  base_serial_port:=/dev/ttyUSB0 \
  micro_ros_baudrate:=921600
```

### Gazebo Simulation

```bash
ros2 launch nikiro_description gazebo.launch.py
```

---

## 🗺️ Mapping & Navigation

### Keyboard Teleoperation

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

| Key | Action |
|-----|--------|
| `i` | Drive forward |
| `,` | Reverse |
| `j` | Rotate CCW |
| `l` | Rotate CW |
| `u` / `o` | Turn (forward + rotate) |
| `m` / `.` | Turn (reverse + rotate) |

### Create a Map (SLAM)

```bash
ros2 launch nikiro_amr_navigation slam.launch.py rviz:=true
```

Move the robot around the environment to generate the map.

### Save the Map

```bash
cd nikiro/src/nikiro_amr/nikiro_amr_navigation/maps
ros2 run nav2_map_server map_saver_cli -f <map_name> \
  --ros-args -p save_map_timeout:=10000
```

### Run Navigation (Nav2)

```bash
ros2 launch linorobot2_navigation navigation.launch.py \
  map:=<map_name.yaml> rviz:=true
```

Set the **2D Pose Estimate** and **Nav2 Goal** in RViz to start autonomous navigation.

---

## 🔧 Custom URDF

To use your own URDF instead of the default Nikiro description, edit the launch file:

```bash
cd nikiro_amr/src/nikiro_amr/nikiro_amr_description/launch
gedit description.launch.py
```

Change the package name and your `.xacro` or `.urdf` filename at **line 26**.

---

## 📄 License

BSD 3-Clause License — see [LICENSE](LICENSE) for details.
