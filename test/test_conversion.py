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

from soccer_vision_3d_msgs.msg import Ball
from soccer_vision_3d_rviz_markers.conversion import ball_to_marker, conf_to_alpha
from soccer_vision_attribute_msgs.msg import Confidence
from visualization_msgs.msg import Marker


def test_conf_to_alpha():

    confidence = Confidence(confidence=Confidence.CONFIDENCE_UNKNOWN)
    assert conf_to_alpha(confidence) == 1.0

    confidence = Confidence(confidence=0.7)
    assert conf_to_alpha(confidence) == 0.7


def test_ball_to_marker():

    ball = Ball()
    ball.center.x = 1.0
    ball.center.y = 2.0
    ball.center.z = 3.0
    ball.confidence.confidence = 0.5
    marker = ball_to_marker(ball)

    assert marker.id == 0
    assert marker.type == Marker.SPHERE
    assert marker.action == Marker.MODIFY
    assert marker.pose.position.x == 1.0
    assert marker.pose.position.y == 2.0
    assert marker.pose.position.z == 3.0
    # assert marker.scale.x == 0.13
    # assert marker.scale.y == 0.13
    # assert marker.scale.z == 0.13
    assert marker.color.r == 1.0
    assert marker.color.g == 0.0
    assert marker.color.b == 0.0
    assert marker.color.a == 0.5
