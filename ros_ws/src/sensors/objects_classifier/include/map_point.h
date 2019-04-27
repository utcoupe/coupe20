#ifndef OBJECTS_CLASSIFIER_MAP_POINT
#define OBJECTS_CLASSIFIER_MAP_POINT

#include <geometry_tools/point.h>

#include <array>

class PointMap: public Point {
public:
    bool is_map;
};

const int SENSORS_NBR = 6;

// maximum number of points created per rect
const int MAX_POINTS = 500;

const std::size_t NB_MAX_POINTS = SENSORS_NBR * MAX_POINTS;

using PointArray = std::array<PointMap, NB_MAX_POINTS>;


#endif // OBJECTS_CLASSIFIER_MAP_POINT
