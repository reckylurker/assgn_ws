# Dynamic Window Approach (DWA) for Turtlebot3 Waffle

This repository contains the code for simulating dynamic window approach (DWA) for Turtlebot3 Waffle in Gazebo. The algorithm logic is based on the amazing robotics library [AtsushiSakai/PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics) and the paper on [The Dynamic Window Approach to Collision Avoidance](https://www.ri.cmu.edu/pub_files/pub1/fox_dieter_1997_1/fox_dieter_1997_1.pdf).

<i>The simulations were performed on WSL2 with Ubuntu-22.04.5 LTS (Jammy), ROS2 Humble, and Gazebo Harmonic.</i>

## Environment Setup

To setup the environment, do the following: 

1. Install ROS2 Humble

Follow the steps [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

> [!NOTE]
> Source ROS2 Installation
```sh
source /opt/ros/humble/setup.bash
```

2. Install Gazebo Harmonic 

Follow the steps [here](https://gazebosim.org/docs/latest/ros_installation/#gazebo-harmonic-with-ros-2-humble)

> [!NOTE]
> Enable GPU Acceleration for NVIDIA GPUs in WSL2
```sh
export NVIDIA_DRIVER_CAPABILITIES=all
export LD_LIBRARY_PATH=/usr/lib/wsl/lib
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
export MESA_LOADER_DRIVER_OVERRIDE=d3d12
export GALLIUM_DRIVER=d3d12
export LIBGL_ALWAYS_INDIRECT=0
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu/dri:/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
```

3. Setup Turtlebot3 Simulations

Follow the steps [here](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/) to complete the initial setup. 

> [!NOTE]
> The Simulation setup provided [here](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation) is incompatible with Gazebo Harmonic. Perform the following: 
```sh
cd ~/turtlebot3_ws/src
git clone -b new_gazebo https://github.com/azeey/turtlebot3_simulations.git
cd ~/turtlebot3_ws
colcon build --symlink-install
source ~/turtlebot3_ws/install/setup.bash
```

4. Setup Custom DWA

```sh
git clone https://github.com/reckylurker/assgn_ws.git && cd assgn_ws
colcon build --packages-select custom_dwa
source install/setup.bash
```

## Running the simulation

> [!NOTE] 
> Ensure that the `custom_dwa` package is sourced in all terminals.
```sh
source assgn_ws/install/setup.bash
```

1. In a terminal, launch `custom_dwa` planner:
```sh
ros2 run custom_dwa launch_dwa
```

2. In another terminal, launch Gazebo and RViz2:
```sh
export TURTLEBOT3_MODEL=waffle
ros2 launch custom_dwa custom_dwa.launch.py
```

- Uncheck `TF` and `Odometry` in RViz2 for better visuals. 
- Add `MakerArray` display with topic `/dwa_trajs`.
- Use `2D Goal Pose` in RViz2 to give navigation commands. 

Watch the robot navigate.

## Debug

In the terminal running `custom_dwa` planner, the DWA Planning commands containg the `cost` and the `bestCmd` chosen. The command is printed as: `(linear velocity, angular velocity)`.

To check published velocity commands:
```sh
ros2 topic echo /cmd_vel
```

## Algorithm Overview

### Dynamic Window Approach (DWA)

The DWA algorithm follows these steps:

1. <b>Dynamic Window Generation</b>: Sample velocity commands (linear and angular) within the robot's dynamic constaints.
2. <b>Trajectory Prediction</b>: For each velocity sample, numerically predict the robot's trajectory over a time horizon. The unicycle dynamics model is used for this prediction.
3. <b>Cost Evaluation</b>: Evaluate each trajectory using a multi-objective cost function:
- <b>Goal Distance</b>: Distance from trajectory endpoint to goal.
- <b>Obstacle Avoidance</b>: Minimum distance to obstacles along trajectory
- <b>Path Smoothness</b>: Velocity and Angular Velocity Magnitudes
4. <b>Best Command Selection</b>: Choose the velocity command with the lowest total cost.

### Cost Function

```math
Cost = w_{goal} C_{goal} + w_{obs} C_{obs} + w_{LV} C_{LV} + w_{AV} C_{AV}
```

where $`C_{goal}`$ represents the goal cost, $`C_{obs}`$ represents obstacle cost, $`C_{LV}`$ represents linear velocity cost, and $`C_{AV}`$ represents angular velocity cost. Here, path smoothness is represented through a velocity-based cost term that penalizes high speeds, as the current task is limited to speed regulation for mobile navigation.

### Configurable Parameters

The configurable parameters can be found in [assgn_ws/src/custom_dwa/custom_dwa/DWAConfig.py](https://github.com/reckylurker/assgn_ws/blob/main/src/custom_dwa/custom_dwa/DWAConfig.py).

## Issues with this implementation

With the current parameter settings, the algorithm rarely manages to recover after colliding with an obstacle. The robot tends to exhibit oscillatory behavior near obstacles, likely due to suboptimal parameter tuning. Given the time constraints of this assignment, extensive tuning was not performed. Currently, The robot should operate correctly for any goal as long as it avoids collisions.

## File Structure

```sh
assgn_turtlebot/
├── README.md
└── src
    └── custom_dwa
        ├── custom_dwa
        │   ├── DWAConfig.py
        │   ├── DWALogging.py
        │   ├── DWANode.py
        │   ├── DWAPlanner.py
        │   ├── dwa.py
        │   └── __init__.py
        ├── launch
        │   └── custom_dwa.launch.py
        ├── package.xml
        ├── resource
        │   └── custom_dwa
        ├── setup.cfg
        ├── setup.py
        └── test
            ├── test_copyright.py
            ├── test_flake8.py
            └── test_pep257.py
```
