#include "collisions/engine/engine.h"

#include <cmath>

namespace CollisionResolver {
    bool _pointInRect(Position pos, const RectObstacle& rect);
    bool _pointInCrcle(Position pos, const CircleObstacle& circ);
    bool _segmentsIntersect(const SegmentObstacle& segment1, const SegmentObstacle& segment2);
    bool _rectsIntersect(const RectObstacle& rect1, const RectObstacle& rect2);
    bool _segmentIntersectRect(const SegmentObstacle& segment, const RectObstacle& rect);
    bool _segmentIntersectCircle(const SegmentObstacle& segment, const CircleObstacle&);
    bool _rectIntersectCircle(const RectObstacle& rect, const CircleObstacle& circle);
    bool _circlesIntersect(const CircleObstacle& circle1, const CircleObstacle& circle2);
}

bool CollisionResolver::_pointInRect(Position pos, const RectObstacle& rect)
{
    // Calculs perso de @MadeInPierre #PS22
    double phi = std::atan2(pos.getY() - rect.getPos().getY(), pos.getX() - rect.getPos().getY());
    if (phi < 0)
        phi += 2*M_1_PI;
    double angle = rect.getPos().getAngle();
//     if (angle < 0)
//         angle += 2*M_PI;
    double dist = pos.norm2Dist(rect.getPos());
    Point localPoint(dist * std::cos(phi - angle), dist * std::sin(phi - angle));
    return (
        std::abs(localPoint.getX()) * 2.0 <= rect.getWidth()
        && std::abs(localPoint.getY()) * 2.0 <= rect.getHeight()
    );
}
