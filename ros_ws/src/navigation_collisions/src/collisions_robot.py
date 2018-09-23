import math
from collisions_engine import Position, Velocity, CollisionLevel, PathCheckZone


class NavStatus(object):
    STATUS_IDLE       = 0
    STATUS_NAVIGATING = 1


class Robot(object): #TODO Inherit from RectObstacle
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self._position = Position(0, 0)
        self._velocity = Velocity(self.width, self.height)
        self._nav_status = NavStatus.STATUS_IDLE

        self._path_check_zone = PathCheckZone(self.width, self.height, CollisionLevel.LEVEL_DANGER)

    def update_position(self, tuple3):
        self._position.x = tuple3[0]
        self._position.y = tuple3[1]
        self._position.a = tuple3[2]

    def update_velocity(self, linear, angular):
        self._velocity.linear = linear
        self._velocity.angular = angular

    def update_status(self, new_status):
        self._nav_status = new_status

    def update_waypoints(self, new_waypoints):
        self._path_check_zone.update_waypoints(new_waypoints)

    def get_main_shapes(self):
        return self._velocity.get_shapes(self._position, self._get_max_main_dist())

    def get_path_shapes(self):
        return self._path_check_zone.get_shapes(self._position)

    def check_collisions(self, obstacles):
        return self._velocity.check_collisions(self._position, obstacles) + \
               self._path_check_zone.check_collisions(self._position, obstacles)
        # TODO remove duplicate collisions between the two

    def _get_max_main_dist(self):
        if isinstance(self._path_check_zone.waypoints, list) and len(self._path_check_zone.waypoints) > 0:
            w = self._path_check_zone.waypoints[0]
            return math.sqrt((w.x - self._position.x) ** 2 + (w.y - self._position.y) ** 2)
        else:
            return -1 # invalid waypoints
