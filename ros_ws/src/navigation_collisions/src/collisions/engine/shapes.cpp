#include "collisions/engine/shapes.h"

SegmentObstacle::SegmentObstacle(const Point& firstPoint, const Point& lastPoint, double velocity = 0.0):
    _first(firstPoint),
    _last(lastPoint),
    _velocity(velocity)
{
    _length = firstPoint.norm2Dist(lastPoint);
    _centerPos = Position(
                    (firstPoint + lastPoint)/2,
                    std::atan2(lastPoint.getY() - firstPoint.getY(), lastPoint.getX() - firstPoint.getX())
                 );
    MapObstacle(_centerPos, velocity);
}
