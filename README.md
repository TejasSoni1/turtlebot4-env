# TurtleBot4 Simulation Monorepo

This repository provides a containerised development environment for
running the TurtleBot4 robot in a headless Gazebo simulation with
Foxglove for visualisation.  All required ROS 2 Humble packages are
installed in Docker images so the only host requirements are Docker and
VS Code (with WSL on Windows).

## Quick Start

1. Install Docker and VS Code.  On Windows, enable WSL2.
2. Clone this repository inside your WSL filesystem.
3. Open the folder in VS Code and select **Reopen in Container** when
   prompted.
4. The devcontainer launches three services via Docker Compose:
   - **robot** – TurtleBot4 node and Nav2 stack
   - **simulation** – Ignition Gazebo running headless with simulated
     LiDAR and odometry
   - **foxglove** – `foxglove_bridge` exposing ROS topics on port `8765`
5. Launch Foxglove Studio and connect to `ws://localhost:8765` to view
   topics such as `/scan` and `/odom`.
6. (Optional) Download `TurtleBotTutorial.ipynb` into `notebooks/` to
   run Python examples that command the robot.


The robot image installs the following packages from the official
TurtleBot 4 repository:

```
ros-humble-turtlebot4-msgs
ros-humble-turtlebot4-navigation
ros-humble-turtlebot4-node
```

The included URDF references `turtlebot4_description` so the simulated
robot matches the real hardware.
7. Alternatively, run `./start_sim.sh` to build and start the stack without VS Code.


The containers use ROS 2 Humble on Ubuntu 22.04 with all TurtleBot4
packages pinned to their apt versions to ensure a stable environment.
