#ifndef CURVETRAJECTORY_H
#define CURVETRAJECTORY_H

#include <vector>
#include <geometry_tools/point.h>
#include "cubicbezier.h"

class CurveTrajectory
{
public:
    CurveTrajectory(std::vector<Point> points);
    std::vector<Point> compute(double tStep);
    std::vector<Point> compute(int nbPoints);
    void setPoints(std::vector<Point> points);
    std::vector<Point> getPathPoints();

private:
    std::vector<Point> _points, _pathPoints;

};

#endif // CURVETRAJECTORY_H
