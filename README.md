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


# Usage

In this example, we will visualize a manually published goalpost in RViz2.

First, run the visualizer node:

```sh
ros2 run soccer_vision_3d_rviz_markers visualizer
```

Set up a publisher that publishes a goalpost:

```sh
ros2 topic pub /soccer_vision_3d/goalposts soccer_vision_3d_msgs/msg/GoalpostArray "
header:
  frame_id: 'map'
posts:
  - bb:
      center:
        position:
          x: 2.0
          y: 0.0
          z: 0.2
        orientation:
          x: 0.0
          y: 0.0
          z: 0.0
          w: 1.0
      size:
        x: 0.05
        y: 0.05
        z: 0.4
    attributes:
      side: 0
      team: 0
    confidence:
      confidence: -1.0
"
```


Open RViz2:

```sh
rviz2
```

Add a MarkerArray, and set the topic to ``/soccer_vision_3d/goalposts``.
