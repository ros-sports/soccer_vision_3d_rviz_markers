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

from soccer_vision_3d_msgs.msg import BallArray
from soccer_vision_3d_rviz_markers.visualizer import SoccerVision3DMarkers

from visualization_msgs.msg import Marker


class TestVisualizer:
    """Tests against the SoccerVision3DMarkers class."""

    received = None

    def _callback_msg(self, msg):
        print("callback!")
        self.received = msg

    def test_topics(self):

        rclpy.init()

        visualizer_node = SoccerVision3DMarkers()

        dict_topics = dict(visualizer_node.get_topic_names_and_types())

        # Check publishers
        assert '/soccer_vision_3d_msgs' in dict_topics
        assert 'visualization_msgs/msg/Marker' in dict_topics['/soccer_vision_3d_msgs']

        # Check subscriptions
        assert '/balls_relative' in dict_topics
        assert 'soccer_vision_3d_msgs/msg/BallArray' in dict_topics['/balls_relative']

        assert '/goal_posts_relative' in dict_topics
        assert 'soccer_vision_3d_msgs/msg/GoalpostArray' in dict_topics['/goal_posts_relative']

        assert '/robots_relative' in dict_topics
        assert 'soccer_vision_3d_msgs/msg/RobotArray' in dict_topics['/robots_relative']

        assert '/field_boundary_relative' in dict_topics
        assert 'soccer_vision_3d_msgs/msg/FieldBoundary' in dict_topics['/field_boundary_relative']

        rclpy.shutdown()

    def test_balls(self):

        rclpy.init()
        visualizer_node = SoccerVision3DMarkers()
        test_node = rclpy.node.Node('test')
        publisher = test_node.create_publisher(BallArray, 'balls_relative', 10)
        subscription = test_node.create_subscription(  # noqa: F841
            Marker, 'soccer_vision_3d_msgs', self._callback_msg, 10)

        # Publish BallArray to visualizer_node
        publisher.publish(BallArray())

        rclpy.spin_once(visualizer_node, timeout_sec=0.1)
        rclpy.spin_once(test_node, timeout_sec=0.1)

        assert self.received is not None

        rclpy.shutdown()
