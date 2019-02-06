#include "geometry_tools/position.h"

std::ostream& operator<<(std::ostream& os, Position pos) {
    os << "(" << pos._x << "," << pos._y << ", " << pos._a <<  ")";
    return os;
}
