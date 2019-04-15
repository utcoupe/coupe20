#include "pathfinder/BarriersSubscribers/objects_classifier_subscriber.h"

#include <cmath>

using namespace std;
using namespace Recognition;

ObjectsClassifierSubscriber::ObjectsClassifierSubscriber(double safetyMargin):
    AbstractBarriersSubscriber(safetyMargin) {
    //
}

bool ObjectsClassifierSubscriber::hasBarrier(const Point& pos)
{
    lock_guard<mutex> lock(g_mutex);
    
    // Rectangles
    for (const auto& rect : lastRects)
        if (isInsideRect(rect, pos))
            return true;
    
    // Circles
    for (const auto& circ : lastCircles)
        if (isInsideCircle(circ, pos))
            return true;
    
    // Segments
    for (const auto& seg : lastSegments)
        if (isCloseToSegment(seg, pos))
            return true;
        
    return false;
}

void ObjectsClassifierSubscriber::subscribe(ros::NodeHandle& nodeHandle, std::size_t sizeMaxQueue, std::string topic)
{
    subscriber = nodeHandle.subscribe(
        topic,
        sizeMaxQueue,
        &Recognition::ObjectsClassifierSubscriber::objectsCallback,
        this
    );
}

void ObjectsClassifierSubscriber::objectsCallback(const Objects::ConstPtr& msg)
{
    lock_guard<mutex> lock(g_mutex);
    lastRects.clear();
    addRects(msg->unknown_rects);
    lastCircles.clear();
    addCircles(msg->unknown_circles);
    lastSegments.clear();
    addSegments(msg->unknown_segments);
}

void ObjectsClassifierSubscriber::addRects(const std::vector<Rectangle>& rects)
{
    lastRects.insert(lastRects.end(), rects.begin(), rects.end());
}

void ObjectsClassifierSubscriber::addCircles(const std::vector<Circle>& circs)
{
    lastCircles.insert(lastCircles.end(), circs.begin(), circs.end());
}

void ObjectsClassifierSubscriber::addSegments(const std::vector<Segment>& segs)
{
    lastSegments.insert(lastSegments.end(), segs.begin(), segs.end());
}

bool ObjectsClassifierSubscriber::isInsideRect(const Rectangle& rect, const Point& pos) const
{
    if (rect.h == 0 || rect.w == 0)
        return false;
    double dx, dy; // we want the center of the rectangle as origin
    dx = pos.getX() - rect.x;
    dy = pos.getY() - rect.y;
    double a, b; // (a,b) => coordinates of pos with the center of the rectangle as origin and its sides as vectors
    a = -dx*cos(rect.a) - dy*sin(M_PI - rect.a) + rect.w/2;
    b = dx*sin(rect.a) - dy*cos(rect.a);
    // if a/rect.witdh  is in [-1/2,1/2] and b/rect.height in [-1/2,1/2], then the pos is inside the rectangle
    double da, db;
    da = a/(rect.w + 2*_safetyMargin);
    db = b/(rect.h + 2*_safetyMargin);
    if (da  > 1.0/2.0 || da < -1.0/2.0)
        return false;
    if (db > 1.0/2.0 || db < -1.0/2.0)
        return false;
    return true;
}

bool ObjectsClassifierSubscriber::isInsideCircle(const Circle& circ, const Point& pos) const
{
    processing_lidar_objects::CircleObstacle circle(circ.circle);
    double distToCenter = sqrt(pow(pos.getX() - circle.center.x, 2) + pow(pos.getY() - circle.center.y, 2));
    return (distToCenter + _safetyMargin <= circle.radius);
}

bool ObjectsClassifierSubscriber::isCloseToSegment(const Segment& seg, const Point& pos) const
{
    Point firstPoint(seg.segment.first_point), lastPoint(seg.segment.last_point);
    if (firstPoint == lastPoint) // It's a point, not a segment
        return false;
    // A and B are the limits of the segment, M is our current position
    Point vectAB = lastPoint - firstPoint;
    Point vectAM = pos - firstPoint;
    Point vectBM = pos - lastPoint;
    double proj = vectAB.scalarProduct(vectAM);
    
    double distToSeg;
    if (proj < 0)
        distToSeg = vectAM.norm2Dist({});
    else if (proj > vectAB.scalarProduct(vectAB))
        distToSeg = vectBM.norm2Dist({});
    else
        distToSeg = abs(vectAB.vectorProduct(vectAM)) / vectAB.norm2Dist({}) ;
    return distToSeg <= _safetyMargin;
}
