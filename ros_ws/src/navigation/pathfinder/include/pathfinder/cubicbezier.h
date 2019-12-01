#ifndef CUBICBEZIER_H
#define CUBICBEZIER_H

#include <vector>
#include <geometry_tools/point.h>

class CubicBezier
{
public:
    CubicBezier();
    CubicBezier(Point P0, Point P1, Point P2, Point P3);
    std::vector<Point> compute(double tStep);
    std::vector<Point> compute(int nbPoints);
    void setP0(Point P);
    void setP1(Point P);
    void setP2(Point P);
    void setP3(Point P);
    void setParameters(Point P0, Point P1, Point P2, Point P3);
    Point getP0();
    Point getP1();
    Point getP2();
    Point getP3();
    std::vector<Point> getPoints();

private:
    Point _P0, _P1, _P2, _P3;
    std::vector<Point> _points;

};

#endif // CUBICBEZIER_H
