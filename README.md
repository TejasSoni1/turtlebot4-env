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
   - **robot** – TurtleBot4 ROS 2 nodes and navigation stack
   - **simulation** – Ignition Gazebo running headless with simulated
     LiDAR and odometry
   - **foxglove** – `foxglove_bridge` exposing ROS topics on port `8765`
5. Launch Foxglove Studio and connect to `ws://localhost:8765` to view
   topics such as `/scan` and `/odom`.
6. (Optional) Download `TurtleBotTutorial.ipynb` into `notebooks/` to
   run Python examples that command the robot.

The containers use ROS 2 Humble on Ubuntu 22.04 with all TurtleBot4
packages pinned to their apt versions to ensure a stable environment.
