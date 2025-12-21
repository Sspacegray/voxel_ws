[日本語](README.ja.md) | [English](README.md)

[![ROS 2 Humble build](https://github.com/kzm784/waypoint_editor/actions/workflows/humble_build.yml/badge.svg?branch=main&label=ROS%202%20Humble%20build)](https://github.com/kzm784/waypoint_editor/actions/workflows/humble_build.yml)
[![ROS 2 Jazzy build](https://github.com/kzm784/waypoint_editor/actions/workflows/jazzy_build.yml/badge.svg?branch=main&label=ROS%202%20Jazzy%20build)](https://github.com/kzm784/waypoint_editor/actions/workflows/jazzy_build.yml)

# Waypoint Editor

![demo](https://raw.github.com/wiki/kzm784/waypoint_editor/images/waypoint_editor_demo.gif)

## Table of Contents
- [Overview](#overview)
- [Development Environment](#development-environment)
- [Installation](#installation)
- [Usage](#usage)
- [License](#License)


## Overview
This package provides a tool for intuitively editing and saving waypoints used in robot navigation while referencing 2D/3D maps.  
The edited waypoints can be saved in **CSV** or **Nav2-compatible YAML** format, and waypoints can also be auto-captured from a localization topic.


## Development Environment
- Ubuntu 22.04 (Jammy Jellyfish)
- ROS 2 Humble Hawksbill


## Installation
Run the following commands in your terminal:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/kzm784/waypoint_editor.git
cd ~/ros2_ws
rosdep update && rosdep install --from-paths src --ignore-src -y
colcon build
```

## Usage
### 1. Launching the Waypoint Editor  
Run the following commands to launch the tool:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch waypoint_editor waypoint_editor.launch.py
```

If you already have a simulator + Nav2 running (so a `map_server` is already started),
launch **RViz with waypoint_editor only** to avoid a duplicate map server:

```bash
ros2 launch waypoint_editor waypoint_editor_rviz.launch.py
```

### 2. Loading a Map (2D / 3D)  
- Use Nav2's `nav2_map_server` to load a 2D map in `.yaml` format.  
- You can also load a 3D map in `.pcd` format.  
- In the RViz panel, use "**Load 2D Map**" for `.yaml` or "**Load 3D Map**" for `.pcd`.

![load_map_demo](https://raw.github.com/wiki/kzm784/waypoint_editor/images/loading_2d_map_demo.gif)


### 3. Adding Waypoints  
- From the toolbar at the top of RViz2, select "**Add Waypoint**".  
- Click and drag on the map to add a new waypoint with the desired position and orientation.  
- Once added, each waypoint can:
  - Be **moved or rotated** via drag operations
  - Be **right-clicked** to open a context menu for deletion or other actions

![adding_waypoints_demo](https://raw.github.com/wiki/kzm784/waypoint_editor/images/Adding_waypoints_demo.gif)


### 4. Auto-capturing Waypoints from Localization (optional)
- In the panel's **Auto Capture** section, set:
  - `Auto Δd`: minimum distance between auto-added waypoints
  - `Topic`: localization topic (e.g., `/amcl_pose`)
  - `Type`: `PoseWithCovarianceStamped` or `PoseStamped`
- Click "**Start Auto Capture**" to begin; waypoints will drop automatically as you move.
- Click again to stop.

### 5. Saving Waypoints  
- Click "**Save WPs**" in the bottom-right panel of RViz2 and pick a format in the dialog.
  - **CSV / YAML**: waypoint list (YAML is Nav2 waypoint-follower compatible)
  - **JSON / TXT**: exports a `pp_controller`-compatible *segment path* JSON (`task_id`, `paths[]`, points in **mm**)

![saving_waypoints_demo](https://raw.github.com/wiki/kzm784/waypoint_editor/images/saving_waypoints.gif)


### 6. Loading Waypoints  
- Click "**Load WPs**" button in the bottom-right panel of RViz2 and select the previously saved `.csv` or `.yaml` file.  
- The waypoints can then be edited again in the same interface.

![loading_waypoints_demo](https://raw.github.com/wiki/kzm784/waypoint_editor/images/loading_waypoints.gif)

### 7. Undo / Redo / Clear  
- Use the panel's "**Undo**" and "**Redo**" buttons to step backward or forward through waypoint edits (moving, rotating, deleting, ID changes, or command edits).
- Use "**Clear All**" to remove all waypoints at once.

### 8. Publish a fixed global path & track it (Nav2 FollowPath)
- "**Publish Path**" publishes a `nav_msgs/Path` to `path_topic` (default: `waypoint_path`).
- "**Nav2: Follow Path**" sends a `nav2_msgs/FollowPath` goal to Nav2's controller server (default action name: `follow_path`).

Parameters (declared on the RViz node):
- `path_topic` (string, default `waypoint_path`)
- `path_interpolation_step_m` (double, default `0.10`)
- `publish_path_on_change` (bool, default `true`)
- `follow_path_action_name` (string, default `follow_path`)
- `follow_path_controller_id` (string, default `FollowPath`)
- `follow_path_goal_checker_id` (string, default `general_goal_checker`)


## License
[![License: Apache-2.0](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)

This project is licensed under the Apache License, Version 2.0.  
See the [LICENSE](LICENSE) file for details.
