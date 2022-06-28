# Copyright 2022 Kenji Brameld
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from soccer_vision_3d_msgs.msg import (
    BallArray, GoalpostArray, MarkingArray, ObstacleArray, RobotArray)
from soccer_vision_3d_rviz_markers.conversion import (
    ball_to_marker, goalpost_to_marker, marking_ellipse_to_marker,
    marking_intersection_to_marker, marking_segment_to_marker, obstacle_to_marker,
    robot_to_markers)
from visualization_msgs.msg import Marker, MarkerArray


def ball_array_to_marker_array(msg: BallArray) -> MarkerArray:
    marker_array = MarkerArray()
    marker_array.markers.append(Marker(action=Marker.DELETEALL))
    for ball in msg.balls:
        marker = ball_to_marker(ball)
        marker.header = msg.header
        marker_array.markers.append(marker)
    return marker_array


def goalpost_array_to_marker_array(msg: GoalpostArray) -> MarkerArray:
    marker_array = MarkerArray()
    marker_array.markers.append(Marker(action=Marker.DELETEALL))
    for goalpost in msg.posts:
        marker = goalpost_to_marker(goalpost)
        marker.header = msg.header
        marker_array.markers.append(marker)
    return marker_array


def marking_array_to_marker_array(msg: MarkingArray) -> MarkerArray:
    marker_array = MarkerArray()
    marker_array.markers.append(Marker(action=Marker.DELETEALL))
    for marking in msg.ellipses:
        marker = marking_ellipse_to_marker(marking)
        marker.header = msg.header
        marker_array.markers.append(marker)
    for marking in msg.intersections:
        marker = marking_intersection_to_marker(marking)
        marker.header = msg.header
        marker_array.markers.append(marker)
    for marking in msg.segments:
        marker = marking_segment_to_marker(marking)
        marker.header = msg.header
        marker_array.markers.append(marker)
    return marker_array


def obstacle_array_to_marker_array(msg: ObstacleArray) -> MarkerArray:
    marker_array = MarkerArray()
    marker_array.markers.append(Marker(action=Marker.DELETEALL))
    for obstacle in msg.obstacles:
        marker = obstacle_to_marker(obstacle)
        marker.header = msg.header
        marker_array.markers.append(marker)
    return marker_array


def robot_array_to_marker_array(msg: RobotArray) -> MarkerArray:
    marker_array = MarkerArray()
    marker_array.markers.append(Marker(action=Marker.DELETEALL))
    for robot in msg.robots:
        markers = robot_to_markers(robot)
        for marker in markers:
            marker.header = msg.header
            marker_array.markers.append(marker)
    return marker_array
