#include "collisions/engine/shapes.h"

#include "collisions/engine/check_zone.h"

#include <cmath>
#include <utility>

void MapObstacle::initializeMapObstacle(Position pos, PtrVelocity velocity) {
    _pos = pos;
    _velocity = velocity;
}


SegmentObstacle::SegmentObstacle(const Point& firstPoint, const Point& lastPoint, PtrVelocity velocity):
    MapObstacle(),
    _first(firstPoint),
    _last(lastPoint)
{
    _velocity = velocity;
    _length = firstPoint.norm2Dist(lastPoint);
    _centerPos = Position(
                    (firstPoint + lastPoint)/2,
                    std::atan2(lastPoint.getY() - firstPoint.getY(), lastPoint.getX() - firstPoint.getX())
                 );
    initializeMapObstacle(_centerPos, velocity);
}

std::vector<SegmentObstacle> RectObstacle::toSegments() const
{
    std::vector<SegmentObstacle> segments;
    auto corners = getCorners();
    for (unsigned idCorner = 0; idCorner + 1 < corners.size(); idCorner++)
        segments.emplace_back(corners[idCorner], corners[idCorner + 1]);
    return segments;
}

std::vector<Point> RectObstacle::getCorners() const
{
    std::vector<Point> corners;
    double len = std::sqrt(std::pow(_width/2, 2) + std::pow(_height/2, 2));
    double cornerPhi = std::atan2(_height, _width);
    for (double anglePhi: {0.0, M_PI}) {
        for (int i: {0, 1}) {
            double phi = anglePhi + ((i + 1)%2 == 0 ? 1 : -1) * cornerPhi;
            corners.emplace_back(
                _pos.getX() + len * std::cos(phi + _pos.getAngle()),
                _pos.getY() + len * std::sin(phi + _pos.getAngle())
            );
        }
    }
    return corners;
}
