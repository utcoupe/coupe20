#include "geometry_tools/position.h"

geometry_msgs::Pose2D Position::toPose2D() const {
    geometry_msgs::Pose2D pos;
    pos.x = _x;
    pos.y = _y;
    pos.theta = _a;
    return pos;
}

std::ostream& operator<<(std::ostream& os, Position pos) {
    os << "(" << pos._x << "," << pos._y << ", " << pos._a <<  ")";
    return os;
}
