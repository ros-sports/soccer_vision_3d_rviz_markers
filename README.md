# Soccer Vision 3D RViz Markers

[![Build and Test (rolling)](https://github.com/ros-sports/soccer_vision_3d_rviz_markers/actions/workflows/build_and_test_rolling.yaml/badge.svg?branch=rolling)](https://github.com/ros-sports/soccer_vision_3d_rviz_markers/actions/workflows/build_and_test_rolling.yaml?query=branch:rolling)

Package that contains libraries to display [soccer_vision_3d_msgs](https://index.ros.org/p/soccer_vision_3d_msgs/) as RViz Markers.

# Installation

Only ROS2 Rolling is currently supported.

## ROS2 Rolling

To build from source, source your ROS installation, then run the following in your ROS workspace:

```sh
git clone https://github.com/ros-sports/soccer_vision_3d_rviz_markers.git src/soccer_vision_3d_rviz_markers --branch ${ROS_DISTRO}
colcon build
```
