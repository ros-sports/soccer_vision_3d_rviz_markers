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
from soccer_vision_3d_rviz_markers.conversion import ball_to_marker
from visualization_msgs.msg import Marker, MarkerArray


class SoccerVision3DMarkers(Node):

    def __init__(self):
        super().__init__('SoccerVision3DMarkers')

        # Create publisher
        self.balls_publisher = self.create_publisher(
            MarkerArray, 'visualization/balls', 10)
        self.goalpost_publisher = self.create_publisher(
            MarkerArray, 'visualization/goalposts', 10)
        self.field_boundary_publisher = self.create_publisher(
            Marker, 'visualization/field_boundary', 10)
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
        pass

    def goalposts_cb(self, msg: GoalpostArray):
        pass

    def markings_cb(self, msg: MarkingArray):
        pass

    def obstacles_cb(self, msg: ObstacleArray):
        pass

    def robots_cb(self, msg: RobotArray):
        pass


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
