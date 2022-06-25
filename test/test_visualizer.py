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

from soccer_vision_3d_msgs.msg import (
    Ball, BallArray, FieldBoundary, Goalpost, GoalpostArray, MarkingArray, MarkingEllipse,
    MarkingIntersection, MarkingSegment, Obstacle, ObstacleArray, Robot, RobotArray)
from soccer_vision_3d_rviz_markers.visualizer import SoccerVision3DMarkers
from visualization_msgs.msg import Marker, MarkerArray


class TestVisualizer:
    """Tests against the SoccerVision3DMarkers class."""

    received = None

    def _callback_msg(self, msg):
        self.received = msg

    def test_topics(self):

        rclpy.init()
        visualizer_node = SoccerVision3DMarkers()
        dict_topics = dict(visualizer_node.get_topic_names_and_types())

        # Check publishers
        assert '/visualization/balls' in dict_topics
        assert 'visualization_msgs/msg/MarkerArray' in dict_topics['/visualization/balls']

        assert '/visualization/field_boundary' in dict_topics
        assert 'visualization_msgs/msg/Marker' in dict_topics['/visualization/field_boundary']

        assert '/visualization/goalposts' in dict_topics
        assert 'visualization_msgs/msg/MarkerArray' in dict_topics['/visualization/goalposts']

        assert '/visualization/markings' in dict_topics
        assert 'visualization_msgs/msg/MarkerArray' in dict_topics['/visualization/markings']

        assert '/visualization/obstacles' in dict_topics
        assert 'visualization_msgs/msg/MarkerArray' in dict_topics['/visualization/obstacles']

        assert '/visualization/robots' in dict_topics
        assert 'visualization_msgs/msg/MarkerArray' in dict_topics['/visualization/robots']

        # Check subscriptions
        assert '/soccer_vision_3d/balls' in dict_topics
        assert 'soccer_vision_3d_msgs/msg/BallArray' in dict_topics['/soccer_vision_3d/balls']

        assert '/soccer_vision_3d/field_boundary' in dict_topics
        assert 'soccer_vision_3d_msgs/msg/FieldBoundary' in \
            dict_topics['/soccer_vision_3d/field_boundary']

        assert '/soccer_vision_3d/goalposts' in dict_topics
        assert 'soccer_vision_3d_msgs/msg/GoalpostArray' in \
            dict_topics['/soccer_vision_3d/goalposts']

        assert '/soccer_vision_3d/markings' in dict_topics
        assert 'soccer_vision_3d_msgs/msg/MarkingArray' in \
            dict_topics['/soccer_vision_3d/markings']

        assert '/soccer_vision_3d/obstacles' in dict_topics
        assert 'soccer_vision_3d_msgs/msg/ObstacleArray' in \
            dict_topics['/soccer_vision_3d/obstacles']

        assert '/soccer_vision_3d/robots' in dict_topics
        assert 'soccer_vision_3d_msgs/msg/RobotArray' in dict_topics['/soccer_vision_3d/robots']

        rclpy.shutdown()

    def test_zero_balls(self):

        rclpy.init()
        visualizer_node = SoccerVision3DMarkers()
        test_node = rclpy.node.Node('test')
        publisher = test_node.create_publisher(
            BallArray, 'soccer_vision_3d/balls', 10)
        subscription = test_node.create_subscription(  # noqa: F841
            MarkerArray, 'visualization/balls', self._callback_msg, 10)

        publisher.publish(BallArray())

        rclpy.spin_once(visualizer_node, timeout_sec=0.1)
        rclpy.spin_once(test_node, timeout_sec=0.1)

        assert self.received is not None
        assert self.received.markers == []

        rclpy.shutdown()

    def test_multiple_balls(self):

        rclpy.init()
        visualizer_node = SoccerVision3DMarkers()
        test_node = rclpy.node.Node('test')
        publisher = test_node.create_publisher(
            BallArray, 'soccer_vision_3d/balls', 10)
        subscription = test_node.create_subscription(  # noqa: F841
            MarkerArray, 'visualization/balls', self._callback_msg, 10)

        publisher.publish(BallArray(balls=[Ball(), Ball()]))

        rclpy.spin_once(visualizer_node, timeout_sec=0.1)
        rclpy.spin_once(test_node, timeout_sec=0.1)

        assert self.received is not None
        assert len(self.received.markers) == 2

        rclpy.shutdown()

    def test_field_boundary(self):

        rclpy.init()
        visualizer_node = SoccerVision3DMarkers()
        test_node = rclpy.node.Node('test')
        publisher = test_node.create_publisher(
            FieldBoundary, 'soccer_vision_3d/field_boundary', 10)
        subscription = test_node.create_subscription(  # noqa: F841
            Marker, 'visualization/field_boundary', self._callback_msg, 10)

        publisher.publish(FieldBoundary())

        rclpy.spin_once(visualizer_node, timeout_sec=0.1)
        rclpy.spin_once(test_node, timeout_sec=0.1)

        assert self.received is not None

        rclpy.shutdown()

    def test_zero_goalposts(self):

        rclpy.init()
        visualizer_node = SoccerVision3DMarkers()
        test_node = rclpy.node.Node('test')
        publisher = test_node.create_publisher(
            GoalpostArray, 'soccer_vision_3d/goalposts', 10)
        subscription = test_node.create_subscription(  # noqa: F841
            MarkerArray, 'visualization/goalposts', self._callback_msg, 10)

        publisher.publish(GoalpostArray())

        rclpy.spin_once(visualizer_node, timeout_sec=0.1)
        rclpy.spin_once(test_node, timeout_sec=0.1)

        assert self.received is not None
        assert self.received.markers == []

        rclpy.shutdown()

    def test_multiple_goalposts(self):

        rclpy.init()
        visualizer_node = SoccerVision3DMarkers()
        test_node = rclpy.node.Node('test')
        publisher = test_node.create_publisher(
            GoalpostArray, 'soccer_vision_3d/goalposts', 10)
        subscription = test_node.create_subscription(  # noqa: F841
            MarkerArray, 'visualization/goalposts', self._callback_msg, 10)

        publisher.publish(GoalpostArray(posts=[Goalpost(), Goalpost()]))

        rclpy.spin_once(visualizer_node, timeout_sec=0.1)
        rclpy.spin_once(test_node, timeout_sec=0.1)

        assert self.received is not None
        assert len(self.received.markers) == 2

        rclpy.shutdown()

    # Repeat test_no_balls for markings

    def test_zero_markings(self):

        rclpy.init()
        visualizer_node = SoccerVision3DMarkers()
        test_node = rclpy.node.Node('test')
        publisher = test_node.create_publisher(
            MarkingArray, 'soccer_vision_3d/markings', 10)
        subscription = test_node.create_subscription(  # noqa: F841
            MarkerArray, 'visualization/markings', self._callback_msg, 10)

        publisher.publish(MarkingArray())

        rclpy.spin_once(visualizer_node, timeout_sec=0.1)
        rclpy.spin_once(test_node, timeout_sec=0.1)

        assert self.received is not None
        assert self.received.markers == []

        rclpy.shutdown()

    # Repeat test_multiple_balls for marking
    def test_multiple_markings(self):

        rclpy.init()
        visualizer_node = SoccerVision3DMarkers()
        test_node = rclpy.node.Node('test')
        publisher = test_node.create_publisher(
            MarkingArray, 'soccer_vision_3d/markings', 10)
        subscription = test_node.create_subscription(  # noqa: F841
            MarkerArray, 'visualization/markings', self._callback_msg, 10)

        publisher.publish(MarkingArray(
            segments=[MarkingSegment(), MarkingSegment()],
            intersections=[MarkingIntersection(), MarkingIntersection()],
            ellipses=[MarkingEllipse(), MarkingEllipse()]))

        rclpy.spin_once(visualizer_node, timeout_sec=0.1)
        rclpy.spin_once(test_node, timeout_sec=0.1)

        assert self.received is not None
        assert len(self.received.markers) == 6

        rclpy.shutdown()

    # Repeat test_no_balls for obstacles

    def test_zero_obstacles(self):

        rclpy.init()
        visualizer_node = SoccerVision3DMarkers()
        test_node = rclpy.node.Node('test')
        publisher = test_node.create_publisher(
            ObstacleArray, 'soccer_vision_3d/obstacles', 10)
        subscription = test_node.create_subscription(  # noqa: F841
            MarkerArray, 'visualization/obstacles', self._callback_msg, 10)

        publisher.publish(ObstacleArray())

        rclpy.spin_once(visualizer_node, timeout_sec=0.1)
        rclpy.spin_once(test_node, timeout_sec=0.1)

        assert self.received is not None
        assert self.received.markers == []

        rclpy.shutdown()

    # Repeat test_multiple_balls for obstacles
    def test_multiple_obstacles(self):

        rclpy.init()
        visualizer_node = SoccerVision3DMarkers()
        test_node = rclpy.node.Node('test')
        publisher = test_node.create_publisher(
            ObstacleArray, 'soccer_vision_3d/obstacles', 10)
        subscription = test_node.create_subscription(  # noqa: F841
            MarkerArray, 'visualization/obstacles', self._callback_msg, 10)

        publisher.publish(ObstacleArray(obstacles=[Obstacle(), Obstacle()]))

        rclpy.spin_once(visualizer_node, timeout_sec=0.1)
        rclpy.spin_once(test_node, timeout_sec=0.1)

        assert self.received is not None
        assert len(self.received.markers) == 2

        rclpy.shutdown()

    # Repeat test_no_balls for robots
    def test_zero_robots(self):

        rclpy.init()
        visualizer_node = SoccerVision3DMarkers()
        test_node = rclpy.node.Node('test')
        publisher = test_node.create_publisher(
            RobotArray, 'soccer_vision_3d/robots', 10)
        subscription = test_node.create_subscription(  # noqa: F841
            MarkerArray, 'visualization/robots', self._callback_msg, 10)

        publisher.publish(RobotArray())

        rclpy.spin_once(visualizer_node, timeout_sec=0.1)
        rclpy.spin_once(test_node, timeout_sec=0.1)

        assert self.received is not None
        assert self.received.markers == []

        rclpy.shutdown()

    # Repeat test_multiple_balls for obstacles
    def test_multiple_robots(self):

        rclpy.init()
        visualizer_node = SoccerVision3DMarkers()
        test_node = rclpy.node.Node('test')
        publisher = test_node.create_publisher(
            RobotArray, 'soccer_vision_3d/robots', 10)
        subscription = test_node.create_subscription(  # noqa: F841
            MarkerArray, 'visualization/robots', self._callback_msg, 10)

        publisher.publish(RobotArray(robots=[Robot(), Robot()]))

        rclpy.spin_once(visualizer_node, timeout_sec=0.1)
        rclpy.spin_once(test_node, timeout_sec=0.1)

        assert self.received is not None
        assert len(self.received.markers) == 2

        rclpy.shutdown()
