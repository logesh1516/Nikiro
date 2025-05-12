# Nikiro
Nikiro is a ROS 2-based Autonomous Mobile Robot (AMR) platform designed for indoor navigation using a differential drive system. Built with modularity and simulation in mind, Nikiro leverages Gazebo, RViz, and Nav2 for SLAM, localization, and path planning.This is the Slam package of my robot.This repo is based on linorobot2.

# INSTALLATION
```
mkdir -p nikiro_amr/src
cd nikiro_amr/src
git clone https://github.com/logesh1516/Nikiro.git
cd ..
colcon build && source install setup.bash
```
#ENVIRONMENT SETUP
```
echo "export NIKIRO_LASER_SENSOR=ydlidar" >> ~/.bashrc
source ~/.bashrc
```
since i used ydlidar X2 , I used ydlidar_ros2_driver.

# HARDWARE SETUP

https://github.com/logesh1516/Nikiro_hardware.git

# URDF
THis is my custom urdf model , which uses differntial drive .

![Screenshot from 2025-05-11 12-57-53](https://github.com/user-attachments/assets/1fb7a25f-acb4-4d96-beb2-737281d3180f)

Custom urdf description can also be used instead of nikiro_description

Your custom urdf can be used by changing in 
```
cd nikiro_amr/src/nikiro_amr/nikiro_amr_description/launch
gedit description.launch.py
```
change the the package name to your custom package name and use your **.xacro or .urdf** file in the line **26**

# Booting up the robot

**Using a real robot:**
```
ros2 launch nikiro_amr_bringup bringup.launch.py
```
Optional parameters:

base_serial_port - Serial port of the robot's microcontroller. The assumed value is /dev/ttyACM0. Otherwise, change the 
default value to the correct serial port. For example:

```
ros2 launch nikiro_amr_bringup bringup.launch.py base_serial_port:=/dev/ttyACM1
```

micro_ros_baudrate - micro-ROS serial baudrate. default 115200.

ros2 launch nikiro_amr_bringup bringup.launch.py base_serial_port:=/dev/ttyUSB0 micro_ros_baudrate:=921600

**Using Gazebo:**
```
ros2 launch nikiro_description gazebo.launch.py
```
**Controlling the robot**

Keyboard Teleop
```
Run teleop_twist_keyboard to control the robot using your keyboard:
```

ros2 run teleop_twist_keyboard teleop_twist_keyboard
Press:

i - To drive the robot forward.

, - To reverse the robot.

j - To rotate the robot CCW.

l - To rotate the robot CW.

shift + j - To strafe the robot to the left (for mecanum robots).

shift + l - To strafe the robot to the right (for mecanum robots).

u / o / m / . - Used for turning the robot, combining linear velocity x and angular velocity z.


**Creating a map**

Run SLAM Toolbox:

ros2 launch nikiro_amr_navigation slam.launch.py rviz:=true

Optional parameters for simulation on host machine:

rviz - Set to true to visualize the robot in RVIZ. Default value is false.

Move the robot to map the environment.

**Save the map**
```
cd nikiro/src/nikiro_amr/nikiro_amr_navigation/maps
ros2 run nav2_map_server map_saver_cli -f <map_name> --ros-args -p save_map_timeout:=10000.
```
**Run Nav2 package:**
```
ros2 launch linorobot2_navigation navigation.launch.py rviz:=true
```
Optional parameter for loading maps:

map - Path to newly created map <map_name.yaml>.
Optional parameters for simulation on host machine:

rviz - Set to true to visualize the robot in RVIZ. Default value is false.

set the pose estimate and goal postion to perform navigation


