#ifndef objects_classifier_SUBSCRIBER
#define objects_classifier_SUBSCRIBER

#include "abstract_barriers_subscriber.h"

#include "objects_classifier/ClassifiedObjects.h"

#include <vector>

namespace Recognition
{
    class ObjectsClassifierSubscriber : public AbstractBarriersSubscriber
    {
    public:
        ObjectsClassifierSubscriber(double safetyMargin);
        
        bool hasBarrier(const geometry_msgs::Pose2D& pos) override;
        void subscribe(ros::NodeHandle& nodeHandle, std::size_t sizeMaxQueue, std::string topic) override;
        
    private:
        using Objects       = objects_classifier::ClassifiedObjects;
        using Rectangle     = belt_interpreter::RectangleStamped;
        using Circle        = objects_classifier::CircleObstacleStamped;
        using Segment       = objects_classifier::SegmentObstacleStamped;
        
        std::vector<Rectangle> lastRects;
        std::vector<Circle> lastCircles;
        std::vector<Segment> lastSegments;
        
        void objectsCallback(const Objects::ConstPtr& msg);
        void addRects(const std::vector<Rectangle>& rects);
        void addCircles(const std::vector<Circle>& circs);
        void addSegments(const std::vector<Segment>& segs);
        
        bool isInsideRect(const Rectangle& rect, const geometry_msgs::Pose2D& pos) const;
        bool isInsideCircle(const Circle& circ, const geometry_msgs::Pose2D& pos) const;
        bool isCloseToSegment(const Segment& seg, const geometry_msgs::Pose2D& pos) const;
        
        double scalarProduct(std::pair<double, double> vA, std::pair<double, double> vB) const;
        double vectorProduct(std::pair<double, double> vA, std::pair<double, double> vB) const;
    };
} // namespace Recognition

#endif // objects_classifier_SUBSCRIBER
