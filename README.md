# Soccer Vision 3D RViz Markers

[![Build and Test (rolling)](https://github.com/ros-sports/soccer_vision_3d_rviz_markers/actions/workflows/build_and_test_rolling.yaml/badge.svg?branch=rolling)](https://github.com/ros-sports/soccer_vision_3d_rviz_markers/actions/workflows/build_and_test_rolling.yaml?query=branch:rolling)

Package that contains libraries to display [soccer_vision_3d_msgs](https://index.ros.org/p/soccer_vision_3d_msgs/) as RViz Markers.

The node simply listens for ball, field boundary, goalposts, markings, obstacles and robots,
converts them to visualization markers and publishes them so they can be visualized in RViz.

# Installation

Only ROS2 Rolling is currently supported.

## ROS2 Rolling

To build from source, source your ROS installation, then run the following in your ROS workspace:

```sh
git clone https://github.com/ros-sports/soccer_vision_3d_rviz_markers.git src/soccer_vision_3d_rviz_markers --branch ${ROS_DISTRO}
rosdep install --from-paths src -i
colcon build
```

# Topics

## Subscriptions

* `/soccer_vision_3d/balls` (`soccer_vision_3d_msgs/BallArray`)
* `/soccer_vision_3d/field_boundary` (`soccer_vision_3d_msgs/FieldBoundary`)
* `/soccer_vision_3d/goalposts` (`soccer_vision_3d_msgs/GoalpostArray`)
* `/soccer_vision_3d/markings` (`soccer_vision_3d_msgs/MarkingArray`)
* `/soccer_vision_3d/obstacles` (`soccer_vision_3d_msgs/ObstacleArray`)
* `/soccer_vision_3d/robots` (`soccer_vision_3d_msgs/RobotArray`)

## Publishers

* `/visualization/balls` (`visualization_msgs/msg/MarkerArray`)
* `/visualization/field_boundary` (`visualization_msgs/msg/Marker`)
* `/visualization/goalposts` (`visualization_msgs/msg/MarkerArray`)
* `/visualization/markings` (`visualization_msgs/msg/MarkerArray`)
* `/visualization/obstacles` (`visualization_msgs/msg/MarkerArray`)
* `/visualization/robots` (`visualization_msgs/msg/MarkerArray`)



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
  frame_id: 'camera'
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


Open RViz2 using a configuration file provided by the package:

```sh
rviz2 -d $(ros2 pkg prefix --share soccer_vision_3d_rviz_markers)/rviz/demo.rviz
```

You should see a white goalpost be displayed in RViz!

