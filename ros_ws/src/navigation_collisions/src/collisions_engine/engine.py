#!/usr/bin/python
import math
import rospy

from engine_constants import CollisionLevel
from engine_shapes import Position, RectObstacle, CircleObstacle


class Collision(object):
    def __init__(self, collision_level, obstacle, approx_distance):
        self.level = collision_level
        self.obstacle = obstacle
        self.approx_distance = approx_distance # Distance between the centers of the robot and obstacle (poor precision)


class CollisionsResolver(object):
    @staticmethod
    def find_collisions(robot_shapes, obstacles_shapes):
        collisions = []
        for robot_shape in robot_shapes:
            for obstacle_shape in obstacles_shapes:
                intersecting = False
                if CollisionsResolver.intersect(robot_shape, obstacle_shape):
                    collisions.append(obstacle_shape)
                    intersecting = True

                if obstacle_shape.velocity is not None and not intersecting: # if the obstacle has a velocity, check its velocity zone too.
                    for vel_shape in obstacle_shape.velocity.get_shapes(obstacle_shape.position):
                        if CollisionsResolver.intersect(robot_shape, vel_shape):
                            collisions.append(vel_shape)
        return collisions

    @staticmethod
    def intersect(obs1, obs2):
        def _segments_intersect(segment1, segment2):
            s1, s2 = (segment1.first, segment1.last), (segment2.first, segment2.last)
            # https://stackoverflow.com/questions/3838329/how-can-i-check-if-two-segments-intersect
            ccw = lambda a, b, c: (c.y-a.y) * (b.x-a.x) > (b.y-a.y) * (c.x-a.x)
            return ccw(s1[0],s2[0],s2[1]) != ccw(s1[1],s2[0],s2[1]) and ccw(s1[0],s1[1],s2[0]) != ccw(s1[0],s1[1],s2[1])

        def _rects_intersect(rect1, rect2):
            if _point_in_rect(rect1.position, rect2) or _point_in_rect(rect2.position, rect1):
                return True # Checks if a rect is completely inside the other one.
            for s1 in rect1.segments(): # If not, check if segments intersect.
                for s2 in rect2.segments():
                    if _segments_intersect(s1, s2):
                        return True
            return False

        def _segment_intersects_rect(segment, rect):
            if _point_in_rect(segment.position, rect):
                return True # Checks if the segment center is inside the rect.
            for rect_s in rect.segments(): # If not, check if segments intersect.
                if _segments_intersect(segment, rect_s):
                    return True
            return False

        def _segment_intersects_circle(segment, circle):
            new_rect = RectObstacle(segment.position, circle.radius * 2, segment.length + 2 * circle.radius)
            return _rect_intersects_circle(new_rect, circle)

        def _point_in_rect(point, rect): # Calculs persos #PS22
            phi = math.atan2(point.y - rect.position.y, point.x - rect.position.x)
            phi += 2 * math.pi if phi < 0 else 0
            a = rect.position.a #if rect.Position.a > 0 else rect.Position.a + 2 * math.pi
            d = math.sqrt((point.x - rect.position.x) ** 2 + (point.y - rect.position.y) ** 2)
            local_point = (d * math.cos(phi - a), d * math.sin(phi - a))
            return - rect.width / 2.0 <= local_point[0] <= rect.width / 2.0 and - rect.height / 2.0 <= local_point[1] <= rect.height / 2.0

        def _point_in_circle(point, circle):
            d = math.sqrt((point.x - circle.position.x) ** 2 + (point.y - circle.position.y) ** 2)
            return d <= circle.radius

        def _rect_intersects_circle(rect, circle):
            new_rect = RectObstacle(rect.position, rect.width + circle.radius * 2, rect.height + circle.radius * 2)
            return _point_in_rect(circle.position, new_rect)

        def _circles_intersect(circle1, circle2):
            d = math.sqrt((circle2.position.x - circle1.position.x) ** 2 + (circle2.position.y - circle1.position.y) ** 2)
            return d <= circle1.radius + circle2.radius

        types = (str(obs1), str(obs2))
        if types[0] == 'rect' and types[1] == 'rect':
            return _rects_intersect(obs1, obs2)
        elif types[0] == 'circle' and types[1] == 'circle':
            return _circles_intersect(obs1, obs2)
        elif types[0] == 'segment' and types[1] == 'segment':
            return _segments_intersect(obs1, obs2)
        elif types[0] == 'point' and types[1] == 'point':
            return False
        elif "rect" in types and "circle" in types:
            if types[0] == "rect":
                return _rect_intersects_circle(obs1, obs2)
            return _rect_intersects_circle(obs2, obs1)
        elif "segment" in types and "rect" in types:
            if types[0] == "segment":
                return _segment_intersects_rect(obs1, obs2)
            return _segment_intersects_rect(obs2, obs1)
        elif "segment" in types and "circle" in types:
            if types[0] == "segment":
                return _segment_intersects_circle(obs1, obs2)
            return _segment_intersects_circle(obs2, obs1)
        elif "rect" in types and "point" in types:
            if types[0] == "rect":
                return _point_in_rect(obs1, obs2)
            return _rect_intersects_circle(obs2, obs1)
        elif "circle" in types and "point" in types:
            if types[0] == "point":
                return _point_in_rect(obs1, obs2)
            return _rect_intersects_circle(obs2, obs1)
        else:
            rospy.logerr("Collisions couldn't check collision between objects of type '{}' and '{}'.".format(types[0], types[1]))
