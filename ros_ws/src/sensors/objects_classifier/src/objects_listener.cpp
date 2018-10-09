#include "objects_listener.h"

void ObjectsListener::on_belt_callback(const BeltRectsConstPtr &rects) {
    this->rects = *rects;
    main_thread_.classify_and_publish_rects(this->rects);
}

void ObjectsListener::on_lidar_callback(const ObstaclesConstPtr &obstacles) {
    this->lidar_obstacles = *obstacles;
    main_thread_.classify_and_publish_lidar_objects(this->lidar_obstacles);
}