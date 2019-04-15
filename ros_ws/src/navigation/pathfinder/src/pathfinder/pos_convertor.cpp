#include "pathfinder/pos_convertor.h"

using namespace std;

Point PosConvertor::fromRosToMapPos (const Point& rosPos) const
{
    double x, y;
    x = rosPos.getX() * (_sizeMap.getX() / _sizeRos.getX());
    y = rosPos.getY() * (_sizeMap.getY() / _sizeRos.getY());
    
    if (_invertedY)
        y = _sizeMap.getY() - y;
    
    return Point(x, y).floor();
}


Point PosConvertor::fromMapToRosPos (const Point& mapPos) const
{
    double x, y;
    x = mapPos.getX() * (_sizeRos.getX() / _sizeMap.getX());
    y = mapPos.getY() * (_sizeRos.getY() / _sizeMap.getY());
    
    if (_invertedY)
        y = _sizeRos.getY() - y;
    
    return {x, y};
}

double PosConvertor::fromMapToRosDistance(double dist) const
{
    double xCoef = _sizeMap.getX()/_sizeRos.getX();
    double yCoef = _sizeMap.getY()/_sizeRos.getY();
    // We assume that the scale on x and y is the same, we take the linear average to have a better precision.
    double coef = (xCoef + yCoef)/2;
    return dist/coef;
}

double PosConvertor::fromRosToMapDistance(double dist) const
{
    double xCoef = _sizeMap.getX()/_sizeRos.getX();
    double yCoef = _sizeMap.getY()/_sizeRos.getY();
    // We assume that the scale on x and y is the same, we take the linear average to have a better precision.
    double coef = (xCoef + yCoef)/2;
    return dist*coef;
}

