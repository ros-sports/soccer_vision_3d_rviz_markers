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

## Parameters

* `ball_diameter` (`float`) - Diameter of the ball in meters.
* `marking_segment_width` (`float`) - Width of a marking segment in meters.

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
      size:
        x: 0.05
        y: 0.05
        z: 0.4
"
```


Open RViz2 using a configuration file provided by the package:

```sh
rviz2 -d $(ros2 pkg prefix --share soccer_vision_3d_rviz_markers)/rviz/demo.rviz
```

You should see a white goalpost be displayed in RViz!

## Other Examples

Here are examples for publishing other types of soccer_vision_3d_msgs.

### Balls

Publisher that publishes a BallArray with one ball with 0.5 confidence (0.5% transparency):

```sh
ros2 topic pub soccer_vision_3d/balls soccer_vision_3d_msgs/msg/BallArray "
header:
  frame_id: 'camera'
balls:
  - center:
      x: 2.0
      y: 0.0
      z: 0.05
    confidence:
      confidence: 0.5
"
```

### Field Boundary

Publisher that publishes a Field Boundary with four points:

```sh
ros2 topic pub soccer_vision_3d/field_boundary soccer_vision_3d_msgs/msg/FieldBoundary "
header:
  frame_id: 'camera'
points:
  - x: 1.5
    y: -1.0
    z: 0.0
  - x: 2.0
    y: 1.0
    z: 0.0
  - x: 1.9
    y: 2.0
    z: 0.0
  - x: 0.5
    y: 2.5
    z: 0.0
confidence:
  confidence: -1.0
"
```

### Markings

Publisher that publishes MarkingArray with all different types of markings:

```sh
ros2 topic pub soccer_vision_3d/markings soccer_vision_3d_msgs/msg/MarkingArray "
header:
  frame_id: 'camera'
ellipses:
  - diameter: 0.1
    center:
      position:
        x: 1.0
        y: 1.0
        z: 0.0
intersections:
  - center:
      x: 1.0
      y: 0.0
      z: 0.0
    num_rays: 2
    rays:
      - x: 1.0
        y: 0.0
        z: 0.0
      - x: 0.0
        y: 1.0
        z: 0.0
segments:
  - start:
      x: 1.0
      y: -1.0
      z: 0.0
    end:
      x: 2.0
      y: -1.0
      z: 0.0
"
```


### Obstacles

Publisher that publishes an ObstacleArray with one obstacle:

```sh
ros2 topic pub soccer_vision_3d/obstacles soccer_vision_3d_msgs/msg/ObstacleArray "
header:
  frame_id: 'camera'
obstacles:
  - bb:
      center:
        position:
          x: 2.0
          y: 0.0
          z: 0.6
      size:
        x: 0.3
        y: 0.5
        z: 1.2
"
```


### Robots

Publisher that publishes a RobotArray with one teammate (green) and one opponent (red):

```sh
ros2 topic pub /soccer_vision_3d/robots soccer_vision_3d_msgs/msg/RobotArray "
header:
  frame_id: 'camera'
robots:
  - bb:
      center:
        position:
          x: 1.0
          y: 0.5
          z: 0.3
      size:
        x: 0.3
        y: 0.3
        z: 0.6
    attributes:
      player_number: 1
      team: 1
  - bb:
      center:
        position:
          x: 1.0
          y: -0.5
          z: 0.2
      size:
        x: 0.2
        y: 0.2
        z: 0.4
    attributes:
      player_number: 3
      team: 2
"
```
