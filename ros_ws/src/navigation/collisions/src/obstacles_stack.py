import time
import rospy

class Map(object):
    Robot = None


class ObstaclesStack():
    OBSTACLES_LIFESPAN = 0.3  # max time in seconds before being considered too old

    BeltPoints = []
    LidarObjects = []
    Enemies = []

    @staticmethod
    def toList():
        return ObstaclesStack.BeltPoints + ObstaclesStack.LidarObjects + ObstaclesStack.Enemies

    @staticmethod
    def updateBeltPoints(new_obstacles):
        ObstaclesStack.BeltPoints = new_obstacles

    @staticmethod
    def updateLidarObjects(new_obstacles):
        ObstaclesStack.LidarObjects = new_obstacles

    @staticmethod
    def updateEnemies(new_obstacles):
        ObstaclesStack.Enemies = new_obstacles

    @staticmethod
    def garbageCollect():
        current_time = time.time()
        for obstacle in ObstaclesStack.BeltPoints:
            if current_time - obstacle.spawn_time > ObstaclesStack.OBSTACLES_LIFESPAN:
                ObstaclesStack.BeltPoints.remove(obstacle)

        for obstacle in ObstaclesStack.LidarObjects:
            if current_time - obstacle.spawn_time > ObstaclesStack.OBSTACLES_LIFESPAN:
                ObstaclesStack.LidarObjects.remove(obstacle)

        for obstacle in ObstaclesStack.Enemies:
            if current_time - obstacle.spawn_time > ObstaclesStack.OBSTACLES_LIFESPAN:
                ObstaclesStack.Enemies.remove(obstacle)
