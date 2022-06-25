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

from geometry_msgs.msg import Point
from soccer_vision_3d_msgs.msg import (
    Ball, FieldBoundary, Goalpost, MarkingEllipse, MarkingIntersection, MarkingSegment, Obstacle,
    Robot)
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
    marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=conf_to_alpha(msg.confidence))
    return marker


def marking_ellipse_to_marker(msg: MarkingEllipse) -> Marker:
    marker = Marker()
    marker.type = Marker.CYLINDER
    marker.pose = msg.center
    marker.scale.x = msg.diameter
    marker.scale.y = msg.diameter
    marker.color = ColorRGBA(g=1.0, b=1.0, a=conf_to_alpha(msg.confidence))
    return marker


def marking_intersection_to_marker(msg: MarkingIntersection) -> Marker:
    marker = Marker()
    marker.type = Marker.LINE_LIST
    marker.pose.position = msg.center
    for ray in msg.rays:
        marker.points.append(Point())
        marker.points.append(
            Point(x=ray.x * 0.1, y=ray.y * 0.1, z=ray.z * 0.1))
    marker.scale.x = 0.01
    marker.color = ColorRGBA(r=1.0, b=1.0, a=conf_to_alpha(msg.confidence))
    return marker


def marking_segment_to_marker(msg: MarkingSegment) -> Marker:
    marker = Marker()
    marker.type = Marker.LINE_STRIP
    marker.points = [msg.start, msg.end]
    marker.scale.x = 0.05
    marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=conf_to_alpha(msg.confidence))
    return marker


def obstacle_to_marker(msg: Obstacle) -> Marker:
    marker = Marker()
    marker.type = Marker.CUBE
    marker.pose = msg.bb.center
    marker.scale = msg.bb.size
    marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=conf_to_alpha(msg.confidence))
    return marker


def robot_to_markers(msg: Robot) -> list[Marker]:
    markers = []

    color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=conf_to_alpha(msg.confidence))
    if msg.attributes.team is msg.attributes.TEAM_OWN:
        color.g = 1.0
    elif msg.attributes.team is msg.attributes.TEAM_OPPONENT:
        color.r = 1.0
    else:
        color.r = 1.0
        color.g = 1.0
        color.b = 1.0

    marker = Marker()
    marker.type = Marker.CUBE
    marker.pose = msg.bb.center
    marker.scale = msg.bb.size
    marker.color = color
    markers.append(marker)

    marker_text = Marker()
    marker_text.type = Marker.TEXT_VIEW_FACING
    marker_text.pose.position.x = msg.bb.center.position.x
    marker_text.pose.position.y = msg.bb.center.position.y
    marker_text.pose.position.z = msg.bb.center.position.z + msg.bb.size.z / 2 + 0.1
    marker_text.pose.orientation = msg.bb.center.orientation
    marker_text.color = color
    markers.append(marker_text)

    return markers