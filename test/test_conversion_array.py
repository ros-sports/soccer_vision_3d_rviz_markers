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

from builtin_interfaces.msg import Time
from soccer_vision_3d_msgs.msg import (
    Ball, BallArray, Goalpost, GoalpostArray, MarkingArray, MarkingEllipse,
    MarkingIntersection, MarkingSegment, Obstacle, ObstacleArray, Robot, RobotArray)
from soccer_vision_3d_rviz_markers.conversion_array import (
    ball_array_to_marker_array, goalpost_array_to_marker_array, marking_array_to_marker_array,
    obstacle_array_to_marker_array, robot_array_to_marker_array)
from std_msgs.msg import Header
from visualization_msgs.msg import Marker


test_header = Header(stamp=Time(sec=100, nanosec=200), frame_id='test')
deleteall_marker = Marker(action=Marker.DELETEALL)


def test_ball_array_to_marker_array_no_balls():
    ball_array = BallArray(header=test_header)
    marker_array = ball_array_to_marker_array(ball_array)

    assert len(marker_array.markers) == 1
    assert marker_array.markers[0] == deleteall_marker


def test_ball_array_to_marker_array_multiple_balls():
    ball_array = BallArray(header=test_header, balls=[Ball(), Ball()])
    marker_array = ball_array_to_marker_array(ball_array)

    assert len(marker_array.markers) == 3
    assert marker_array.markers[0] == deleteall_marker
    assert marker_array.markers[1].header == test_header


def test_goalpost_array_to_marker_array_no_posts():
    goalpost_array = GoalpostArray(header=test_header)
    marker_array = goalpost_array_to_marker_array(goalpost_array)

    assert len(marker_array.markers) == 1
    assert marker_array.markers[0] == deleteall_marker


def test_goalpost_array_to_marker_array_multiple_posts():
    goalpost_array = GoalpostArray(header=test_header, posts=[
                                   Goalpost(), Goalpost()])
    marker_array = goalpost_array_to_marker_array(goalpost_array)

    assert len(marker_array.markers) == 3
    assert marker_array.markers[0] == deleteall_marker
    assert marker_array.markers[1].header == test_header


def test_marking_array_to_marker_array_no_markings():
    marking_array = MarkingArray(header=test_header)
    marker_array = marking_array_to_marker_array(marking_array)

    assert len(marker_array.markers) == 1
    assert marker_array.markers[0] == deleteall_marker


def test_marking_array_to_marker_array_multiple_markings():
    marking_array = MarkingArray(
        header=test_header,
        ellipses=[MarkingEllipse(), MarkingEllipse()],
        intersections=[MarkingIntersection(), MarkingIntersection()],
        segments=[MarkingSegment(), MarkingSegment()])
    marker_array = marking_array_to_marker_array(marking_array)

    assert len(marker_array.markers) == 7
    assert marker_array.markers[0] == deleteall_marker
    assert marker_array.markers[1].header == test_header


def test_obstacle_array_to_marker_array_no_obstacles():
    obstacle_array = ObstacleArray(header=test_header)
    marker_array = obstacle_array_to_marker_array(obstacle_array)

    assert len(marker_array.markers) == 1
    assert marker_array.markers[0] == deleteall_marker


def test_obstacle_array_to_marker_array_multiple_obstacles():
    obstacle_array = ObstacleArray(
        header=test_header,
        obstacles=[Obstacle(), Obstacle()])
    marker_array = obstacle_array_to_marker_array(obstacle_array)

    assert len(marker_array.markers) == 3
    assert marker_array.markers[0] == deleteall_marker
    assert marker_array.markers[1].header == test_header


def test_robot_array_to_marker_array_no_robots():
    robot_array = RobotArray(header=test_header)
    marker_array = robot_array_to_marker_array(robot_array)

    assert len(marker_array.markers) == 1
    assert marker_array.markers[0] == deleteall_marker


def test_robot_array_to_marker_array_multiple_robots():
    robot_array = RobotArray(header=test_header, robots=[Robot(), Robot()])
    marker_array = robot_array_to_marker_array(robot_array)

    # (number of markers >= number of robots) because a robot can have more than one Marker
    assert len(marker_array.markers) >= 3
    assert marker_array.markers[0] == deleteall_marker
    assert marker_array.markers[1].header == test_header
