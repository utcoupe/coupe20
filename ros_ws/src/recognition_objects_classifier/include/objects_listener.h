
#ifndef RECOGNITION_OBJECTS_CLASSIFIER_OBJECTS_LISTENER_H
#define RECOGNITION_OBJECTS_CLASSIFIER_OBJECTS_LISTENER_H

#include "ros/ros.h"
#include "main_thread.h"
#include "processing_belt_interpreter/BeltRects.h"
#include "processing_lidar_objects/Obstacles.h"

#include <string>

using namespace processing_belt_interpreter;
using namespace processing_lidar_objects;

const std::string BELT_TOPIC = "/processing/belt_interpreter/rects";
const std::string LIDAR_TOPIC = "/processing/lidar_objects/obstacles";


class ObjectsListener {
protected:
    ros::Subscriber belt_sub_;
    ros::Subscriber lidar_sub_;
    MainThread &main_thread_;

    void on_belt_callback(const BeltRectsConstPtr &rects);

    void on_lidar_callback(const ObstaclesConstPtr &obstacles);

public:
    BeltRects rects;
    Obstacles lidar_obstacles;

    ObjectsListener(ros::NodeHandle &nh, MainThread &main_thread) :
            main_thread_(main_thread),
            belt_sub_(nh.subscribe(BELT_TOPIC, 1, &ObjectsListener::on_belt_callback, this)),
            lidar_sub_(nh.subscribe(LIDAR_TOPIC, 1, &ObjectsListener::on_lidar_callback, this)) {}

};

#endif //RECOGNITION_OBJECTS_CLASSIFIER_OBJECTS_LISTENER_H
