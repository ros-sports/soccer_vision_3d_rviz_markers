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

from soccer_vision_3d_msgs.msg import Ball, FieldBoundary, Goalpost
from soccer_vision_attribute_msgs.msg import Confidence
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker


def conf_to_alpha(conf: Confidence) -> float:
    if conf.confidence == Confidence.CONFIDENCE_UNKNOWN:
        return 1.0
    return conf.confidence


def ball_to_marker(msg: Ball) -> Marker:
    marker = Marker()
    marker.type = Marker.SPHERE
    marker.pose.position = msg.center
    marker.color = ColorRGBA(r=1.0, a=conf_to_alpha(msg.confidence))
    return marker


def field_boundary_to_marker(msg: FieldBoundary) -> Marker:
    marker = Marker()
    marker.type = Marker.LINE_STRIP
    marker.points = msg.points
    marker.scale.x = 0.02
    marker.color = ColorRGBA(g=1.0, a=conf_to_alpha(msg.confidence))
    return marker


def goalpost_to_marker(msg: Goalpost) -> Marker:
    marker = Marker()
    marker.type = Marker.CYLINDER
    marker.pose = msg.bb.center
    marker.scale = msg.bb.size
    marker.color = ColorRGBA(a=conf_to_alpha(msg.confidence))
    return marker
