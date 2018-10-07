#include "collisions/point.h"

#include <cmath>

using namespace std;

double Point::norm1Dist (const Point& other) const
{
    return std::abs (_x - other.getX ()) + std::abs (_y - other.getY ());
}

double Point::norm2Dist(const Point& other) const
{
    double dX2 = pow(_x - other.getX (), 2);
    double dY2 = pow(_y - other.getY (), 2);
    return sqrt(dX2 + dY2);
}

Point Point::operator+(const Point& other) const
{
    return {_x + other.getX (), _y + other.getY ()};
}

Point Point::operator/ (double divisor) const {
    return {_x / divisor, _y / divisor};
}

bool Point::operator==(const Point& other) const
{
    return norm1Dist(other) == 0;
}

bool Point::operator!=(const Point& other) const
{
    return norm1Dist(other) != 0;
}

ostream & operator<<(ostream& os, const Point& pos)
{
    os << "(" << pos._x << "," << pos._y << ")";
    return os;
}

