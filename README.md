# Create3 Navigation Task

This project contains a custom navigation stack for the Create3 robot, enabling it to navigate through waypoints, locate a docking station, and perform docking with fallback mechanisms. The navigation logic includes advanced features such as an S-shaped path, a 360-degree scan for the docking station, and Fibonacci spiral exploration if the docking station is out of view.

## Table of Contents
1. [Project Overview](#project-overview)
2. [Features](#features)
3. [Installation and Setup](#installation-and-setup)
4. [Usage](#usage)
5. [Navigation Logic](#navigation-logic)
6. [File Structure](#file-structure)
7. [Future Enhancements](#future-enhancements)

---

## Project Overview

This project is developed to control the Create3 robot with ROS2 using custom navigation logic. The robot can undock, navigate through defined waypoints in an S-shaped path, search for a docking station with a 360-degree scan, and perform a Fibonacci spiral search pattern if necessary to locate the docking station.

## Features

- **Custom Waypoint Navigation**: Allows the robot to follow a specified path, including an S-shaped trajectory.
- **360-degree Scan**: Upon reaching the final goal, the robot performs a full scan to locate the docking station.
- **Fibonacci Spiral Exploration**: If the docking station is not detected during the scan, the robot performs a Fibonacci spiral search to increase the chance of locating the station.
- **Docking Procedure**: The robot automatically initiates docking once the station is detected.

## Installation and Setup

### Prerequisites

- ROS2 Humble
- Gazebo (for simulation)
- [Create3 Simulation Packages](https://github.com/iRobotEducation/create3_sim)
- Python 3.8 or higher

### Installation Steps

1. **Clone the Required Repository**:
    - Clone the [Create3 simulation repository](https://github.com/iRobotEducation/create3_sim) by iRobot:
      ```bash
      git clone https://github.com/iRobotEducation/create3_sim.git
      cd create3_sim
      ```

2. **Copy the `create3_navigation_task` Folder**:
    - Copy the `create3_navigation_task` directory into the `create3_sim/irobot_create_gazebo` folder:
      ```bash
      cp -r path/to/your/create3_navigation_task path/to/your/create3_sim/irobot_create_gazebo/
      ```

3. **Set the Ignition Version**:
    - Set the environment variable for Ignition to `fortress`:
      ```bash
      export IGNITION_VERSION=fortress
      ```

4. **Build the Workspace**:
    - Use `colcon` to build the entire workspace:
      ```bash
      colcon build --symlink-install
      source install/local_setup.bash
      ```

5. **Configure Parameters**:
    - Update the `target_pose.yaml` file in `config/` to set custom navigation goals.

## Usage

1. **Launch the Empty World in Gazebo**:
    ```bash
    ros2 launch create3_gazebo_bringup gazebo_empty_world.launch.py
    ```

2. **Run the Navigation Task**:
    Open two separate terminals and run the following commands:
    - In Terminal 1:
      ```bash
      ros2 run create3_navigation_task navigate_create3
      ```
    - In Terminal 2:
      ```bash
      ros2 run create3_navigation_task goal_handling
      ```

## Navigation Logic

### 1. **Undocking and Waypoint Navigation**
   The robot starts by undocking and navigates through waypoints defined in an S-shape pattern, in order to reach the target defined in the `target_pose.yaml` configuration file.

### 2. **360-degree Docking Station Scan**
   After reaching the final waypoint, the robot rotates to perform a 360-degree scan for the docking station. If detected, it proceeds to dock.

### 3. **Fibonacci Spiral Exploration**
   If the docking station is not found during the scan, the robot initiates a Fibonacci spiral to explore the area. It continues to check for the docking station throughout the spiral process, initiating docking once detected.

## File Structure

- **`launch/navigate_launch.py`**: Launch file for initializing the navigation stack and waypoints.
- **`config/`**:
  - **`target_pose.yaml`**: Specifies the target coordinates.
  - **`nav2_params.yaml`**: Configures navigation parameters for the robot.
- **`navigate_create3.py`**: Main script that controls undocking, waypoint navigation, docking scan, and spiral exploration.
- **`goal_handler.py`**: Utility node to handle the goal-based movement of the robot.

## Future Enhancements

- **Obstacle Avoidance**: Integrate obstacle detection and avoidance for safer navigation.
- **Dynamic Waypoints**: Enable real-time waypoint adjustment based on external input.
- **Improved Docking Detection**: Enhance detection accuracy to minimize docking time.

---

## Acknowledgments

Thanks to [iRobot Education](https://github.com/iRobotEducation) for the Create3 simulation packages and the ROS community for the `nav2` navigation stack and support.
