#include "geometry_tools/point.h"

std::ostream& operator<< (std::ostream& os, Point pos) {
    os << "(" << pos._x << "," << pos._y << ")";
    return os;
}
