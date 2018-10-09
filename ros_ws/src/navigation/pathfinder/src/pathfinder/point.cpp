#include "pathfinder/point.h"

using namespace std;

int Point::norm1Dist (const Point& other) const
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
    return Point (_x + other.getX (), _y + other.getY ());
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

