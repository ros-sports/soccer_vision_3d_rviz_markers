# Copyright 2022 Florian Vahl
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

import rclpy
from rclpy.node import Node

from soccer_vision_3d_msgs.msg import (
    BallArray, FieldBoundary, GoalpostArray, MarkingArray, ObstacleArray, RobotArray)
from soccer_vision_3d_rviz_markers.conversion import (
    ball_to_marker, field_boundary_to_marker, goalpost_to_marker, marking_ellipse_to_marker,
    marking_intersection_to_marker, marking_segment_to_marker, obstacle_to_marker, robot_to_marker)
from visualization_msgs.msg import Marker, MarkerArray


class SoccerVision3DMarkers(Node):

    def __init__(self):
        super().__init__('SoccerVision3DMarkers')

        # Create publisher
        self.balls_publisher = self.create_publisher(
            MarkerArray, 'visualization/balls', 10)
        self.field_boundary_publisher = self.create_publisher(
            Marker, 'visualization/field_boundary', 10)
        self.goalpost_publisher = self.create_publisher(
            MarkerArray, 'visualization/goalposts', 10)
        self.goalposts_publisher = self.create_publisher(
            MarkerArray, 'visualization/goalposts', 10)
        self.markings_publisher = self.create_publisher(
            MarkerArray, 'visualization/markings', 10)
        self.obstacles_publisher = self.create_publisher(
            MarkerArray, 'visualization/obstacles', 10)
        self.robots_publisher = self.create_publisher(
            MarkerArray, 'visualization/robots', 10)

        # Create subscriptions
        self.create_subscription(
            BallArray, 'soccer_vision_3d/balls', self.balls_cb, 10)
        self.create_subscription(
            FieldBoundary, 'soccer_vision_3d/field_boundary', self.field_boundary_cb, 10)
        self.create_subscription(
            GoalpostArray, 'soccer_vision_3d/goalposts', self.goalposts_cb, 10)
        self.create_subscription(
            MarkingArray, 'soccer_vision_3d/markings', self.markings_cb, 10)
        self.create_subscription(
            ObstacleArray, 'soccer_vision_3d/obstacles', self.obstacles_cb, 10)
        self.create_subscription(
            RobotArray, 'soccer_vision_3d/robots', self.robots_cb, 10)

    def balls_cb(self, msg: BallArray):
        marker_array = MarkerArray()
        for ball in msg.balls:
            marker_array.markers.append(ball_to_marker(ball))
        self.balls_publisher.publish(marker_array)

    def field_boundary_cb(self, msg: FieldBoundary):
        marker = field_boundary_to_marker(msg)
        self.field_boundary_publisher.publish(marker)

    def goalposts_cb(self, msg: GoalpostArray):
        marker_array = MarkerArray()
        for goalpost in msg.posts:
            marker_array.markers.append(goalpost_to_marker(goalpost))
        self.goalposts_publisher.publish(marker_array)

    def markings_cb(self, msg: MarkingArray):
        marker_array = MarkerArray()
        for marking in msg.ellipses:
            marker_array.markers.append(marking_ellipse_to_marker(marking))
        for marking in msg.intersections:
            marker_array.markers.append(marking_intersection_to_marker(marking))
        for marking in msg.segments:
            marker_array.markers.append(marking_segment_to_marker(marking))
        self.markings_publisher.publish(marker_array)

    def obstacles_cb(self, msg: ObstacleArray):
        obstacles_array = MarkerArray()
        for obstacle in msg.obstacles:
            obstacles_array.markers.append(obstacle_to_marker(obstacle))
        self.obstacles_publisher.publish(obstacles_array)

    def robots_cb(self, msg: RobotArray):
        robots_array = MarkerArray()
        for robot in msg.robots:
            robots_array.markers.append(robot_to_marker(robot))
        self.robots_publisher.publish(robots_array)


def main(args=None):
    rclpy.init(args=args)

    node = SoccerVision3DMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
