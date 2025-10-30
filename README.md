What am I doing? 

Since robotic simulations can be nasty, especially with ROS and Gazebo, I am maintaining a simple todo list to ensure sanity, save time, and avoid going too deep in the rabbit hole. Since Gazebo Classic is too old, and Gazebo Ignition was too bad (too messed up to work with), I am going forward with Gazebo Harmonic.

<i>All the work here will be done on a WSL2 Ubuntu 22.04 instance on Ros2 Humble and Gazebo Harmonic</i>

Todo:
1. Install Ros2 Humble (Done)
- Follow the steps [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).
2. Setup GPU Acceleration (Done)
- For WSL, the following lines can be useful:
```sh
# Gazebo GPU Acceleration FIX (WSL2)
export NVIDIA_DRIVER_CAPABILITIES=all
export LD_LIBRARY_PATH=/usr/lib/wsl/lib
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
export MESA_LOADER_DRIVER_OVERRIDE=d3d12
export GALLIUM_DRIVER=d3d12
export LIBGL_ALWAYS_INDIRECT=0
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu/dri:/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH

# ROS2 Setup
. /opt/ros/humble/setup.bash
```
3. Install Gazebo Harmonic (Done)
- Follow the steps [here](https://gazebosim.org/docs/latest/ros_installation/#gazebo-harmonic-with-ros-2-humble).
4. Setup Turtlebot for Gazebo Harmonic (Done)
- Complete the initial setup [here](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/). Then do the following:
```sh
cd ~/turtlebot3_ws/src
git clone -b new_gazebo https://github.com/azeey/turtlebot3_simulations.git
cd ~/turtlebot3_ws
colcon build --symlink-install
source ~/turtlebot3_ws/install/setup.bash
```

and the part 1 of the assignment is over. To test the simple environment, run the following: 
```sh
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py
```
