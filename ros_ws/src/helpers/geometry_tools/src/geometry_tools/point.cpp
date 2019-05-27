#include "geometry_tools/point.h"

geometry_msgs::Pose2D Point::toPose2D() const {
    geometry_msgs::Pose2D pos;
    pos.x = _x;
    pos.y = _y;
    pos.theta = 0.0;
    return pos;
}

geometry_msgs::Point Point::toGeoPoint() const {
    geometry_msgs::Point pos;
    pos.x = _x;
    pos.y = _y;
    pos.z = 0;
    return pos;
}

std::ostream& operator<< (std::ostream& os, Point pos) {
    os << "(" << pos._x << "," << pos._y << ")";
    return os;
}
