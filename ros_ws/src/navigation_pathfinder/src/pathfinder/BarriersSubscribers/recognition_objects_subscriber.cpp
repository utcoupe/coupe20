#include "pathfinder/BarriersSubscribers/recognition_objects_classifier_subscriber.h"

#include <cmath>

using namespace std;
using namespace Recognition;

ObjectsClassifierSubscriber::ObjectsClassifierSubscriber(const double& safetyMargin)
    : AbstractBarriersSubscriber(safetyMargin)
{
    //
}

bool ObjectsClassifierSubscriber::hasBarrier(const geometry_msgs::Pose2D& pos)
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

bool ObjectsClassifierSubscriber::isInsideRect(const Rectangle& rect, const geometry_msgs::Pose2D& pos) const
{
    if (rect.h == 0 || rect.w == 0)
        return false;
    double dx, dy; // we want the center of the rectangle as origin
    dx = pos.x - rect.x;
    dy = pos.y - rect.y;
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

bool ObjectsClassifierSubscriber::isInsideCircle(const Circle& circ, const geometry_msgs::Pose2D& pos) const
{
    processing_lidar_objects::CircleObstacle circle(circ.circle);
    double distToCenter = sqrt(pow(pos.x - circle.center.x, 2) + pow(pos.y - circle.center.y, 2));
    return (distToCenter + _safetyMargin <= circle.radius);
}

bool ObjectsClassifierSubscriber::isCloseToSegment(const Segment& seg, const geometry_msgs::Pose2D& pos) const
{
    if (seg.segment.first_point.x == seg.segment.last_point.x && seg.segment.first_point.y == seg.segment.last_point.y)
        return false;
    // A and B are the limits of the segment, M is our current position
    pair<double, double> vectAB(seg.segment.last_point.x - seg.segment.first_point.x, seg.segment.last_point.y - seg.segment.first_point.y);
    pair<double, double> vectAM(pos.x - seg.segment.first_point.x, pos.y - seg.segment.first_point.y);
    pair<double, double> vectBM(pos.x - seg.segment.last_point.x, pos.y - seg.segment.last_point.y);
    double proj = scalarProduct(vectAB, vectAM);
    
    double distToSeg;
    if (proj < 0)
        distToSeg = sqrt(scalarProduct(vectAM, vectAM));
    else if (proj > scalarProduct(vectAB, vectAB))
        distToSeg = sqrt(scalarProduct(vectBM, vectBM));
    else
        distToSeg = abs(vectorProduct(vectAB, vectAM))/(sqrt(scalarProduct(vectAB, vectAB)));
    return distToSeg <= _safetyMargin;
}

double ObjectsClassifierSubscriber::scalarProduct(pair<double, double> vA, pair<double, double> vB) const
{
    return vA.first*vB.first + vA.second*vB.second;
}

double ObjectsClassifierSubscriber::vectorProduct(pair<double, double> vA, pair<double, double> vB) const
{
    return vA.first*vB.second - vA.second*vB.first;
}
