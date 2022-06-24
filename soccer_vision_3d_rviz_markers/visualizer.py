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

from soccer_vision_3d_msgs.msg import BallArray, FieldBoundary, GoalpostArray, RobotArray
from visualization_msgs.msg import Marker


class SoccerVision3DMarkers(Node):

    def __init__(self):
        super().__init__('SoccerVision3DMarkers')

        # Create publisher
        self.marker_publisher = self.create_publisher(
            Marker, 'soccer_vision_3d_msgs', 10)

        # Create subscriptions
        self.create_subscription(
            BallArray, 'balls_relative', self.balls_cb, 10)
        self.create_subscription(
            GoalpostArray, 'goal_posts_relative', self.goalpost_cb, 10)
        self.create_subscription(
            RobotArray, 'robots_relative', self.robot_cb, 10)
        self.create_subscription(
            FieldBoundary, 'field_boundary_relative', self.field_boundary_cb, 10)

    def balls_cb(self, msg: BallArray):
        self.marker_publisher.publish(Marker())

    def goalpost_cb(self, msg: GoalpostArray):
        pass

    def robot_cb(self, msg: RobotArray):
        pass

    def field_boundary_cb(self, msg: FieldBoundary):
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
