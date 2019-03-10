#include "geometry_tools/point.h"

geometry_msgs::Pose2D Point::toPose2D() const {
    geometry_msgs::Pose2D pos;
    pos.x = _x;
    pos.y = _y;
    return pos;
}

std::ostream& operator<< (std::ostream& os, Point pos) {
    os << "(" << pos._x << "," << pos._y << ")";
    return os;
}
