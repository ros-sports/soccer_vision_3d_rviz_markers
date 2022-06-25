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

from soccer_vision_3d_msgs.msg import Ball, Goalpost
from soccer_vision_3d_rviz_markers.conversion import (
    ball_to_marker, conf_to_alpha, goalpost_to_marker)
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


def test_goalpost_to_marker():
    goalpost = Goalpost()
    goalpost.bb.center.position.x = 0.1
    goalpost.bb.center.position.y = 0.2
    goalpost.bb.center.position.z = 0.3
    goalpost.bb.center.orientation.x = 0.128
    goalpost.bb.center.orientation.y = 0.145
    goalpost.bb.center.orientation.z = 0.269
    goalpost.bb.center.orientation.w = 0.944
    goalpost.bb.size.x = 0.4
    goalpost.bb.size.y = 0.5
    goalpost.bb.size.z = 0.6
    goalpost.confidence.confidence = 0.7
    marker = goalpost_to_marker(goalpost)

    assert marker.pose.position.x == 0.1
    assert marker.pose.position.y == 0.2
    assert marker.pose.position.z == 0.3
    assert marker.pose.orientation.x == 0.128
    assert marker.pose.orientation.y == 0.145
    assert marker.pose.orientation.z == 0.269
    assert marker.pose.orientation.w == 0.944
    assert marker.scale.x == 0.4
    assert marker.scale.y == 0.5
    assert marker.scale.z == 0.6
    assert marker.color.r == 0.0
    assert marker.color.g == 0.0
    assert marker.color.b == 0.0
    assert marker.color.a == 0.7
