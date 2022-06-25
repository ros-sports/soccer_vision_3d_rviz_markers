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

from geometry_msgs.msg import Point, Vector3
from soccer_vision_3d_msgs.msg import (
    Ball, FieldBoundary, Goalpost, MarkingEllipse, MarkingIntersection, MarkingSegment)
from soccer_vision_3d_rviz_markers.conversion import (
    ball_to_marker, conf_to_alpha, field_boundary_to_marker, goalpost_to_marker,
    marking_ellipse_to_marker, marking_intersection_to_marker, marking_segment_to_marker)
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


def test_fieldboundary_to_marker():
    field_boundary = FieldBoundary()
    field_boundary.points = [Point(x=0.1, y=0.2, z=0.3), Point(x=0.4, y=0.5, z=0.6)]
    field_boundary.confidence.confidence = 0.6
    marker = field_boundary_to_marker(field_boundary)

    assert marker.type == Marker.LINE_STRIP
    assert marker.action == Marker.MODIFY
    assert marker.points[0].x == 0.1
    assert marker.points[0].y == 0.2
    assert marker.points[0].z == 0.3
    assert marker.points[1].x == 0.4
    assert marker.points[1].y == 0.5
    assert marker.points[1].z == 0.6
    assert marker.scale.x == 0.02
    assert marker.color.r == 0.0
    assert marker.color.g == 1.0
    assert marker.color.b == 0.0
    assert marker.color.a == 0.6


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

    assert marker.type == Marker.CYLINDER
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
    assert marker.color.r == 1.0
    assert marker.color.g == 1.0
    assert marker.color.b == 1.0
    assert marker.color.a == 0.7


def test_marking_ellipse_to_marker():
    marking_ellipse = MarkingEllipse()
    marking_ellipse.diameter = 0.5
    marking_ellipse.center.position.x = 0.1
    marking_ellipse.center.position.y = 0.2
    marking_ellipse.center.position.z = 0.3
    marking_ellipse.center.orientation.x = 0.128
    marking_ellipse.center.orientation.y = 0.145
    marking_ellipse.center.orientation.z = 0.269
    marking_ellipse.center.orientation.w = 0.944
    marking_ellipse.confidence.confidence = 0.7
    marker = marking_ellipse_to_marker(marking_ellipse)

    assert marker.type == Marker.CYLINDER
    assert marker.pose.position.x == 0.1
    assert marker.pose.position.y == 0.2
    assert marker.pose.position.z == 0.3
    assert marker.pose.orientation.x == 0.128
    assert marker.pose.orientation.y == 0.145
    assert marker.pose.orientation.z == 0.269
    assert marker.pose.orientation.w == 0.944
    assert marker.scale.x == 0.5  # Should be same as diameter
    assert marker.scale.y == 0.5  # Should be same as diameter
    assert marker.scale.z == 0.0
    assert marker.color.r == 0.0
    assert marker.color.g == 1.0
    assert marker.color.b == 1.0
    assert marker.color.a == 0.7


def test_marking_intersection_to_marker():
    marking_intersection = MarkingIntersection()
    marking_intersection.center.x = 0.1
    marking_intersection.center.y = 0.2
    marking_intersection.center.z = 0.3
    marking_intersection.num_rays = 2
    marking_intersection.rays = [Vector3(x=1.0), Vector3(y=1.0)]
    marking_intersection.confidence.confidence = 0.7
    marker = marking_intersection_to_marker(marking_intersection)

    assert marker.type == Marker.LINE_LIST
    assert marker.pose.position.x == 0.1
    assert marker.pose.position.y == 0.2
    assert marker.pose.position.z == 0.3
    assert Point(x=0.1) in marker.points  # 0.1m length vector in direction of 1st ray
    assert Point(y=0.1) in marker.points  # 0.1m length vector in direction of 2nd ray
    assert marker.points.count(Point()) == 2  # center
    assert marker.scale.x == 0.01  # 0.01m line width
    assert marker.color.r == 1.0
    assert marker.color.g == 0.0
    assert marker.color.b == 1.0
    assert marker.color.a == 0.7


def test_marking_segment_to_marker():
    marking_segment = MarkingSegment()
    marking_segment.start.x = 0.1
    marking_segment.start.y = 0.2
    marking_segment.start.z = 0.3
    marking_segment.end.x = 0.4
    marking_segment.end.y = 0.5
    marking_segment.end.z = 0.6
    marking_segment.confidence.confidence = 0.7
    marker = marking_segment_to_marker(marking_segment)

    assert marker.type == marker.LINE_STRIP
    assert marker.points[0].x == 0.1
    assert marker.points[0].y == 0.2
    assert marker.points[0].z == 0.3
    assert marker.points[1].x == 0.4
    assert marker.points[1].y == 0.5
    assert marker.points[1].z == 0.6
    assert marker.scale.x == 0.05  # 0.05m line width
    assert marker.color.r == 1.0
    assert marker.color.g == 1.0
    assert marker.color.b == 1.0
    assert marker.color.a == 0.7
