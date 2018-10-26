#include "collisions/engine/engine.h"

#include <ros/console.h>

#include <cmath>

namespace CollisionResolver {
    bool _pointInRect(Position pos, const RectObstacle * rect);
    bool _pointInCircle(Position pos, const CircleObstacle * circ);
    bool _segmentsIntersect(const SegmentObstacle* segment1, const SegmentObstacle* segment2);
    bool _rectsIntersect(const RectObstacle* rect1, const RectObstacle* rect2);
    bool _segmentIntersectRect(const SegmentObstacle* segment, const RectObstacle* rect);
    bool _segmentIntersectCircle(const SegmentObstacle* segment, const CircleObstacle* circle);
    bool _rectIntersectCircle(const RectObstacle* rect, const CircleObstacle* circle);
    bool _circlesIntersect(const CircleObstacle* circle1, const CircleObstacle* circle2);
}

std::vector<PtrObstacle> CollisionResolver::findCollisions(const std::vector<PtrObstacle>& robotShapes, const std::vector<PtrObstacle>& obstacleShapes) {
    std::vector<PtrObstacle> collisions;
    for (auto& robotShape: robotShapes) {
        for (auto& obstShape: obstacleShapes) {
            bool intersecting = false;
            if (CollisionResolver::intersect(robotShape, obstShape)) {
                collisions.push_back(obstShape);
                intersecting = true;
            }
            if (!intersecting && obstShape->getVelocity()) {
                //
            }
        }
    }
    return collisions;
}

bool CollisionResolver::intersect(PtrObstacle obst1, PtrObstacle obst2) {
    ObstacleTypes type1, type2;
    type1 = obst1->getTypeName();
    type2 = obst2->getTypeName();
    bool response = false;
    try {
        if (type1 == ObstacleTypes::RECTANGLE) {
            switch(type2) {
                case ObstacleTypes::RECTANGLE:
                    response = _rectsIntersect(
                        dynamic_cast<RectObstacle*>(obst1.get()),
                        dynamic_cast<RectObstacle*>(obst2.get())
                    );
                    break;
                case ObstacleTypes::CIRCLE:
                    response = _rectIntersectCircle(
                        dynamic_cast<RectObstacle*>(obst1.get()),
                        dynamic_cast<CircleObstacle*>(obst2.get())
                    );
                    break;
                case ObstacleTypes::SEGMENT:
                    response = _segmentIntersectRect(
                        dynamic_cast<SegmentObstacle*>(obst2.get()),
                        dynamic_cast<RectObstacle*>(obst1.get())
                    );
                    break;
                default:
                    ROS_WARN("Tried to cast an undefined obsacle!");
            }
        }
        else if (type1 == ObstacleTypes::CIRCLE) {
            switch(type2) {
                case ObstacleTypes::RECTANGLE:
                    response = _rectIntersectCircle(
                        dynamic_cast<RectObstacle*>(obst2.get()),
                        dynamic_cast<CircleObstacle*>(obst1.get())
                    );
                    break;
                case ObstacleTypes::CIRCLE:
                    response = _circlesIntersect(
                        dynamic_cast<CircleObstacle*>(obst1.get()),
                        dynamic_cast<CircleObstacle*>(obst2.get())
                    );
                    break;
                case ObstacleTypes::SEGMENT:
                    response = _segmentIntersectCircle(
                        dynamic_cast<SegmentObstacle*>(obst2.get()),
                        dynamic_cast<CircleObstacle*>(obst1.get())
                    );
                    break;
                default:
                    ROS_WARN("Tried to cast an undefined obsacle!");
            }
        }
        else if (type1 == ObstacleTypes::SEGMENT) {
            switch(type2) {
                case ObstacleTypes::RECTANGLE:
                    response = _segmentIntersectRect(
                        dynamic_cast<SegmentObstacle*>(obst1.get()),
                        dynamic_cast<RectObstacle*>(obst2.get())
                    );
                    break;
                case ObstacleTypes::CIRCLE:
                    response = _segmentIntersectCircle(
                        dynamic_cast<SegmentObstacle*>(obst1.get()),
                        dynamic_cast<CircleObstacle*>(obst2.get())
                    );
                    break;
                case ObstacleTypes::SEGMENT:
                    response = _segmentsIntersect(
                        dynamic_cast<SegmentObstacle*>(obst1.get()),
                        dynamic_cast<SegmentObstacle*>(obst2.get())
                    );
                    break;
                default:
                    ROS_WARN("Tried to cast an undefined obsacle!");
            }
        }
        else {
            ROS_WARN("Tried to cast an undefined obsacle!");
        }
    } catch (const std::bad_cast& err) {
        ROS_ERROR_STREAM("Error when casting obstacles: " << err.what());
    } catch (...) {
        ROS_ERROR("Undefined throw when checking obstacles");
    }
    return response;
}

bool CollisionResolver::_pointInRect(Position pos, const RectObstacle* rect)
{
    // Calculs perso de @MadeInPierre #PS22
    double phi = std::atan2(pos.getY() - rect->getPos().getY(), pos.getX() - rect->getPos().getY());
    if (phi < 0)
        phi += 2*M_1_PI;
    double angle = rect->getPos().getAngle();
//     if (angle < 0)
//         angle += 2*M_PI;
    double dist = pos.norm2Dist(rect->getPos());
    Point localPoint(dist * std::cos(phi - angle), dist * std::sin(phi - angle));
    return (
        std::abs(localPoint.getX()) * 2.0 <= rect->getWidth()
        && std::abs(localPoint.getY()) * 2.0 <= rect->getHeight()
    );
}


bool CollisionResolver::_pointInCircle(Position pos, const CircleObstacle* circ)
{
    double dist = pos.norm2Dist(circ->getPos());
    return dist <= circ->getRadius();
}

bool CollisionResolver::_segmentsIntersect(const SegmentObstacle* segment1, const SegmentObstacle* segment2)
{
    // https://stackoverflow.com/questions/3838329/how-can-i-check-if-two-segments-intersect
    auto ccw = [](Point a, Point b,  Point c) {
        return (
            (c.getY() - a.getY()) * (b.getX() - a.getX())
            > (b.getY() - a.getY()) * (c.getX() - a.getX())
        );
    };
    
    auto seg1 = segment1->segment();
    auto seg2 = segment2->segment();
    
    return (
        ccw(seg1.first, seg2.first, seg2.second) != ccw(seg1.second, seg2.first, seg2.second)
        && ccw(seg1.first, seg1.second, seg2.first) != ccw(seg1.first, seg1.second, seg2.second)
    );
}

bool CollisionResolver::_rectsIntersect(const RectObstacle* rect1, const RectObstacle* rect2)
{
    if (_pointInRect(rect1->getPos(), rect2) || _pointInRect(rect2->getPos(), rect1))
        return true;
    for (const auto seg1: rect1->toSegments())
        if (_segmentIntersectRect(&seg1, rect2))
            return true;
    return false;
}

bool CollisionResolver::_segmentIntersectRect(const SegmentObstacle* segment, const RectObstacle* rect)
{
    if (_pointInRect(segment->getPos(), rect))
        return true;
    for (const auto seg2: rect->toSegments())
        if (_segmentsIntersect(segment, &seg2))
            return true;
    return false;
}

bool CollisionResolver::_segmentIntersectCircle(const SegmentObstacle* segment, const CircleObstacle* circle)
{
    RectObstacle newRect(
        segment->getPos(),
        circle->getRadius() * 2,
        segment->getLength() + 2 * circle->getRadius()
    );
    return _rectIntersectCircle(&newRect, circle);
}

bool CollisionResolver::_rectIntersectCircle(const RectObstacle* rect, const CircleObstacle* circle)
{
    RectObstacle newRect(
        rect->getPos(),
        rect->getWidth() + circle->getRadius() * 2,
        rect->getHeight() + circle->getRadius() * 2
    );
    return _pointInRect(circle->getPos(), &newRect);
}


bool CollisionResolver::_circlesIntersect(const CircleObstacle* circle1, const CircleObstacle* circle2)
{
    double dist = circle1->getPos().norm2Dist(circle2->getPos());
    return dist <= circle1->getRadius() + circle2->getRadius();
}

