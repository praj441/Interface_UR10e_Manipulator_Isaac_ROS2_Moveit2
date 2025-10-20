# Interfacing UR manipulator arm in Isaac sim with ROS2 and Moveit2

In this repository, I will let you go through simulating a UR10e robot in Isaac Sim and then interface it with ROS2 (Humble) and MoveIt2.  
The guidelines will help you simulate your own robot and perform the necessary customizations you need.

---

## System Setup

The following setup was tested on:
- **OS:** Ubuntu 22.04  
- **GPU:** NVIDIA (8 GB VRAM)  
- **RAM:** 32 GB  

Installed:
- **Isaac Sim 5.0.0** in a virtual environment (Python 3.11) provided by `uv` ([Installation Instruction](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/pip_installation.html))
- **ROS2 Humble** at the system level (Python 3.10) ([Installation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html))

---

## Environment Configuration

Since Isaac Sim (Python 3.11) and ROS2 (Python 3.10) require different environments, use the following configurations:

### Isaac Sim Environment
```bash
source ~/env_isaaclab/bin/activate
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/path_to_uv_virtual_env/env_isaaclab/lib/python3.11/site-packages/isaacsim/exts/isaacsim.ros2.bridge/humble/lib
```

you can store these command in a file_name.sh file and do:
```bash
source file_name.sh
```

### ROS:
```bash
source /opt/ros/humble/setup.bash
```

### Ros2 and isaacsim bridging:
Make sure Isaacsim has enabled ROS Bridge package (enabled by default as such). 
You can always check its status: 
Go to the extension manager menu Window > Extensions and search for ROS 2 bridge.
<img src="assets/isaac_ros2_bridge.png" />

### Isaac_ros2_workspace:
Download the isaac_ros2 workspace from below and use its humble_ws folder as your ROS workspace (since I am using ROS2 humble)
https://github.com/isaac-sim/IsaacSim-ros_workspaces/tree/IsaacSim-5.0.0

### topic_based_ros2_control
Isaac-sim (in virtual env) and ROS (system) communicates using [topic_based_ros2_control](https://github.com/PickNikRobotics/topic_based_ros2_control) package.
Download it and move it to yout humble_ws/src folder.


### UR_ROS2 description and driver:
Download the original official drivers from below links and put them in humble_ws/src folder. You need to modify certain things to work it out with Isaac (explained later)
https://github.com/UniversalRobots/Universal_Robots_ROS2_Description
https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver


### Building the humble_ws workspace:
1) cd to /humble_ws
2) rosdep install --from-paths src --ignore-src -r -y
3) colcon build --symlink-install
4) source install/setup.bash   (you need to run this each time you open a new terminal)

### Overall workflow:
1) Run isaac sim and load UR10.usd from (Top-left -> Create -> Robots -> Asset Browser)
2) You need to create an ActionGrasp and configure a few things (I followed this (https://youtu.be/pGje2slp6-s))
3) Run the simulation in Isaac_sim and check if it publish appropiate topics
4) launch moveit2 along with rviz, ros2_control, topi_based_ros2_control 

For moveit part above, refer my code as an example and you need to do following changes:
1) In /src/Universal_Robots_ROS2_Driver-humble/ur_moveit_config/ur_moveit_isaac.launch.py
.
.
.

In progress...
Stay Tuned...
