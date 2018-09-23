import math
import rospy
from engine_constants import CollisionThresholds
from engine_shapes import RectObstacle, CircleObstacle
from engine_shapes_attrib import Position
from engine import Collision, CollisionLevel, CollisionsResolver


class CheckZone(object):
    def __init__(self, width, height, collision_level):
        self._width = width
        self._height = height
        self.collision_level = collision_level

    def get_shapes(self, robot_pos):
        return []

    def check_collisions(self, robot_pos, obstacles):
        raise NotImplementedError("Must be overwritten.")


class VelocityCheckZone(CheckZone):
    def __init__(self, width, height, collision_level):
        super(VelocityCheckZone, self).__init__(width, height, collision_level)

    def get_shapes(self, robot_pos, vel_linear, vel_angular, max_dist = -1):
        if abs(vel_linear) < CollisionThresholds.VEL_MIN: # if the object isn't moving fast enough, don't create the rect.
            return []

        expansion_dist = CollisionThresholds.get_stop_distance(vel_linear)
        if max_dist != -1:
            expansion_dist = min(expansion_dist, max_dist) # If set, reduce the expansion to the provided limit.
        w, h = self._height + expansion_dist, self._width
        l = w / 2.0 - self._height / 2.0

        side_a = math.pi if vel_linear < 0 else 0
        return [RectObstacle(Position(robot_pos.x + l * math.cos(robot_pos.a + side_a),
                                      robot_pos.y + l * math.sin(robot_pos.a + side_a),
                                      robot_pos.a), w, h)]

    def check_collisions(self, robot_pos, vel_linear, vel_angular, obstacles):
        collisions = []
        for o in CollisionsResolver.find_collisions(self.get_shapes(robot_pos, vel_linear, vel_angular), obstacles):
            approx_d = math.sqrt((robot_pos.x - o.position.x) ** 2 + \
                                 (robot_pos.y - o.position.y) ** 2) # Very approximate distance
            collisions.append(Collision(CollisionLevel.LEVEL_STOP, o, approx_d))
        return collisions


class Velocity(object):
    def __init__(self, width, height, linear = 0.0, angular = 0.0):
        self.linear = linear
        self.angular = angular
        self._check_zone = VelocityCheckZone(width, height, CollisionLevel.LEVEL_STOP)

    def get_shapes(self, object_pos, max_dist = -1):
        return self._check_zone.get_shapes(object_pos, self.linear, self.angular, max_dist)

    def check_collisions(self, object_pos, obstacles): # Used only for the robot itself, not obstacles.
        return self._check_zone.check_collisions(object_pos, self.linear, self.angular, obstacles)


class PathCheckZone(CheckZone):
    def __init__(self, width, height, collision_level):
        super(PathCheckZone, self).__init__(width, height, collision_level)
        self.waypoints = []

    def update_waypoints(self, new_waypoints):
        if isinstance(new_waypoints, list) and len(new_waypoints) > 0:
            self.waypoints = new_waypoints
        else:
            rospy.logerr("Trying to update the robot path with an invalid variable type.")

    def _get_full_waypoints(self, robot_pos):
        return [robot_pos] + self.waypoints

    def get_shapes(self, robot_pos):
        if len(self.waypoints) >= 1:
            shapes = []
            path = self._get_full_waypoints(robot_pos)
            for i in range(1, len(path)):
                # Creating a rectangle with the robot's width between each waypoint
                p_w = robot_pos if i == 0 else path[i - 1]
                w   = path[i]

                if p_w.x != w.x and p_w.y != w.y:
                    d = math.sqrt( (w.x - p_w.x) ** 2 + (w.y - p_w.y) ** 2)
                    angle = math.atan((w.y - p_w.y) / (w.x - p_w.x))
                    pos = Position((w.x + p_w.x) / 2.0, (w.y + p_w.y) / 2.0, angle = angle)

                    shapes.append(RectObstacle(pos, d, self._width))
                    if i == len(path) - 1:
                        shapes.append(RectObstacle(Position(w.x, w.y, angle), self._height, self._width))
                    else:
                        r = math.sqrt(self._width ** 2 + self._height ** 2) / 2.0
                        shapes.append(CircleObstacle(Position(w.x, w.y), r))
            return shapes
        else:
            return []

    def check_collisions(self, robot_pos, obstacles):
        collisions = []
        for o in CollisionsResolver.find_collisions(self.get_shapes(robot_pos), obstacles):
            approx_d = math.sqrt((robot_pos.x - o.position.x) ** 2 + \
                                 (robot_pos.y - o.position.y) ** 2) # Very approximate distance
            collisions.append(Collision(CollisionLevel.LEVEL_DANGER if approx_d < CollisionThresholds.DANGER_RADIUS else CollisionLevel.LEVEL_POTENTIAL,
                                        o, approx_d))
        return collisions
