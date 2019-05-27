
#ifndef PROJECT_MAP_OBJECTS_H
#define PROJECT_MAP_OBJECTS_H

#include <ros/ros.h>

#include "shapes.h"

#include <geometry_tools/point.h>

#include <string>
#include <vector>

class MapObjects {
protected:
    std::vector<std::shared_ptr<const Shape>> map_shapes_;
    ros::NodeHandle &nh_;

public:
    float WALLS_MARGIN = 0.03;

    void fetch_map_objects();

    bool contains_point(Point point) const;

    MapObjects(ros::NodeHandle &nh) :
            nh_(nh) {}

};

#endif //PROJECT_MAP_OBJECTS_H
