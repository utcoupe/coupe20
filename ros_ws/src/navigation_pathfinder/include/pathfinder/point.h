#ifndef POINT_H
#define POINT_H

#include <cmath>
#include <ostream>

#include "geometry_msgs/Pose2D.h"

/**
 * Class representing a position or a vector.
 * Have somme basic operators.
 */
class Point
{
public:
    Point (int x, int y) : _x(x), _y(y) {}
    Point () : Point(0,0) {}
    
    int norm1Dist (const Point& other) const;
    double norm2Dist(const Point& other) const;
    
    // Operators
    Point operator+ (const Point& other) const;
    bool operator== (const Point& other) const;
    bool operator!= (const Point& other) const;
    friend std::ostream& operator<< (std::ostream& os, const Point& pos);
    
    // Getters & Setters
    int getX () const { return _x; }
    void setX (const int& x) { _x = x; }
    int getY () const { return _y; }
    void setY (const int& y) { _y = y; }
    
private:
    int _x, _y;
};

#endif // POINT_H
