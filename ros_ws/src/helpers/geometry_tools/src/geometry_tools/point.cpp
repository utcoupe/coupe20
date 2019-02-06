#include "geometry_tools/point.h"

std::ostream& operator<< (std::ostream& os, const Point& pos) {
    os << "(" << pos._x << "," << pos._y << ")";
    return os;
}
