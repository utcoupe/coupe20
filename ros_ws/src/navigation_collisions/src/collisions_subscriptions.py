import math, json
import rospy
import tf2_ros, tf

from obstacles_stack import Map, ObstaclesStack
from collisions_robot import Robot
from collisions_engine import Point, Position, Velocity, SegmentObstacle, RectObstacle, CircleObstacle

from memory_map.srv import MapGet
from navigation_navigator.msg import Status
from drivers_ard_asserv.msg import RobotSpeed
from recognition_objects_classifier.msg import ClassifiedObjects
from ai_game_manager import StatusServices


class CollisionsSubscriptions(object):
    def __init__(self):
        # Preparing to get the robot's position, belt frame_id transform.
        self._tf2_pos_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self._tf2_pos_listener = tf2_ros.TransformListener(self._tf2_pos_buffer)
        self._transform_listener = tf.TransformListener()

        # Callback buffers
        self._nav_status = 0 #STATUS_IDLE
        self._robot_path_waypoints = []
        self._vel_linear  = 0.0
        self._vel_angular = 0.0

        # Subscribing to dependencies
        rospy.Subscriber("/navigation/navigator/status", Status, self._on_nav_status)
        rospy.Subscriber("/recognition/objects_classifier/objects", ClassifiedObjects, self._on_classifier)
        rospy.Subscriber("/drivers/ard_asserv/speed", RobotSpeed, self.on_robot_speed)

        self.game_status = StatusServices("navigation", "collisions", None, self._on_game_status)

    def send_init(self, success = True):
        self.game_status.ready(success)

    def create_robot(self):
        try: # Getting the robot shape and creating the robot instance
            robot_type = rospy.get_param("/robot").lower()
            map_get_client = rospy.ServiceProxy("/memory/map/get", MapGet)
            map_get_client.wait_for_service(2.0)
            shape = json.loads(map_get_client("/entities/" + robot_type + "/shape/*").response)
            if not shape["type"] == "rect":
                raise ValueError("Robot shape type not supported here.")
        except Exception as e:
            rospy.logerr("ERROR Collisions couldn't get the robot's shape from map : " + str(e))
            shape = {"width": 0.4, "height": 0.25}
        return Robot(shape["width"], shape["height"]) # Can create a rect or circle

    def update_robot(self):
        new_pos = self._update_robot_pos()
        if new_pos is not None:
            Map.Robot.update_position(new_pos)

        if self._nav_status is not None:
            Map.Robot.update_status(self._nav_status)
        if self._robot_path_waypoints is not None and len(self._robot_path_waypoints) > 0:
            Map.Robot.update_waypoints(self._robot_path_waypoints)

        Map.Robot.update_velocity(self._vel_linear, self._vel_angular)

    def _update_robot_pos(self):
        try:
            t = self._tf2_pos_buffer.lookup_transform("map", "robot", rospy.Time())
            tx, ty = t.transform.translation.x, t.transform.translation.y
            rz = self._quaternion_to_euler_angle(t.transform.rotation)[2]
            return (tx, ty, rz)
        except Exception as e:
            return None

    def _on_game_status(self, msg):
        pass

    def _on_nav_status(self, msg):
        self._nav_status = msg.status
        self._robot_path_waypoints = [Point(point.x, point.y) for point in msg.currentPath]

    def _on_classifier(self, msg):
        new_belt = []
        for rect in msg.unknown_rects:
            if rect.header.frame_id not in ["map", "/map"]:
                rospy.logwarn("Belt rect not in /map tf frame, skipping.")
                continue

            new_belt.append(RectObstacle(Position(rect.x, rect.y,\
                                                  rect.a),\
                                                  rect.w, rect.h))

        if len(new_belt) > 0:
            ObstaclesStack.updateBeltPoints(new_belt)


        new_lidar = []

        for segment in msg.unknown_segments:
            if segment.header.frame_id not in ["map", "/map"]:
                rospy.logwarn("Lidar segment not in /map tf frame, skipping.")
                continue

            new_lidar.append(SegmentObstacle(Position(segment.segment.first_point.x, segment.segment.first_point.y),
                                             Position(segment.segment.last_point.x,  segment.segment.last_point.y)))
        for circle in msg.unknown_circles:
            if circle.header.frame_id not in ["map", "/map"]:
                rospy.logwarn("Lidar circle not in /map tf frame, skipping.")
                continue
            vel_d = math.sqrt(circle.circle.velocity.y ** 2 + circle.circle.velocity.x ** 2)
            vel_a = math.atan2(circle.circle.velocity.y, circle.circle.velocity.x)
            new_lidar.append(CircleObstacle(Position(circle.circle.center.x, circle.circle.center.y, angle = vel_a),
                                            circle.circle.radius, velocity = Velocity(circle.circle.radius * 2, circle.circle.radius * math.sqrt(3.0) / 2.0,
                                            vel_d, 0.0)))

        if len(new_lidar) > 0:
            ObstaclesStack.updateLidarObjects(new_lidar)

    def on_robot_speed(self, msg):
        self._vel_linear = msg.linear_speed
        self._vel_angular = 0.0

    def _quaternion_to_euler_angle(self, quaternion):
        # https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        w, x, y, z = quaternion.w, quaternion.x, quaternion.y, quaternion.z
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y ** 2)
        X = math.degrees(math.atan2(t0, t1))

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.degrees(math.asin(t2))

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y ** 2 + z * z)
        Z = math.atan2(t3, t4)
        return (X, Y, Z)
