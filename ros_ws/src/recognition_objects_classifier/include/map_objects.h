
#ifndef PROJECT_MAP_OBJECTS_H
#define PROJECT_MAP_OBJECTS_H

#include <string>
#include <vector>

#include <ros/ros.h>

#include "shapes.h"


const std::string MAP_GET_SERVICE = "/memory/map/get";
const std::string MAP_OBJECTS = "/terrain/walls/layer_belt/*";

class MapObjects {
protected:
    std::vector<std::shared_ptr<const Shape>> map_shapes_;
    ros::NodeHandle &nh_;

public:
    float WALLS_MARGIN = 0.03;

    void fetch_map_objects();

    bool contains_point(float x, float y);

    MapObjects(ros::NodeHandle &nh) :
            nh_(nh) {}

};

#endif //PROJECT_MAP_OBJECTS_H
