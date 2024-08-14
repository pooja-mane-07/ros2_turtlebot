# Turtlesim Catch Them All

This ROS 2 project is a game-like application where you control a turtle within the `turtlesim` simulation to `catch` other turtles that are randomly spawned. The project is organized into three main packages: `turtle_bringup`, `turtle_catcher`, and `turtle_interface`.

## Overview of Packages

1. **turtle_bringup** : Provides the launch files to bring up the entire system, including the `turtlesim_node`, `turtle_spawner`, and `turtle_controller` nodes.

2. **turtle_catcher** : Contains the main logic for controlling the turtle and spawning additional turtles.

3. **turtle_interface**: Defines custom message and service types used by the `turtle_spawner` and `turtle_controller` nodes.

## Requirements

- ROS2 (humble or later)
- turtlesim package

## Installation

Clone this repository into your ROS2 workspace and build it using colcon:

```bash
cd ~/ros2_ws/src
git clone https://github.com/pooja-mane-07/turtle_catcher.git
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Usage

Run the Turtle Catcher node:

```bash
ros2 launch turtle_bringup turtle_catcher.launch.py
```