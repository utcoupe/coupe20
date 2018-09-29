#ifndef PATHFINDER_ROS_INTERFACE_H
#define PATHFINDER_ROS_INTERFACE_H

#include <ros/console.h>

#include "pathfinder/pathfinder.h"
#include "pathfinder/dynamic_barriers_manager.h"
#include "pathfinder/pos_convertor.h"

#include "geometry_msgs/Pose2D.h"

class PathfinderROSInterface
{
public:
    PathfinderROSInterface(const std::string& mapFileName, std::shared_ptr<PosConvertor> convertor);
    
    /**
     * Callback for the ros FindPath service. Coordinates are converted between the outside and inside referential.
     * @param req The request, containing the start and end positions.
     * @param rep The response, will contain the shortest path if it exists.
     */
    bool findPathCallback(navigation_pathfinder::FindPath::Request &req, navigation_pathfinder::FindPath::Response &rep);
    
    /**
     * Callback for ros PathfinderNodeConfig dynamic reconfigure service.
     * @param config The new configuration to use.
     * @param level *Not used.*
     */
    void reconfigureCallback(navigation_pathfinder::PathfinderNodeConfig &config, uint32_t level);
    
    void addBarrierSubscriber(DynamicBarriersManager::BarriersSubscriber && subscriber);
    
private:
    std::unique_ptr<Pathfinder> pathfinderPtr_;
    
    std::shared_ptr<DynamicBarriersManager> dynBarriersMng_;
    
    /** Convertor object between inside and outside referentials **/
    std::shared_ptr<PosConvertor> convertor_;
    
    // Convertors
    /**
     * Converts a position from the outside referential and type to the inside ones.
     * @param pos The position in the outside referential and type.
     * @return The position in the inside referential and type.
     */
    Point pose2DToPoint_(const geometry_msgs::Pose2D& pos) const;
    /**
     * Converts a position from the inside referential and type to the outside ones.
     * @param pos The position in the inside referential and type.
     * @return The position in the outside referential and type.
     */
    geometry_msgs::Pose2D pointToPose2D_(const Point& pos) const;
    
    /**
     * Convert the path in the outside type to a string for debugging purposes.
     * @param path The path in outside referential and type.
     * @return The path in string format.
     */
    std::string pathRosToStr_(const std::vector<geometry_msgs::Pose2D>& path);
};

#endif // PATHFINDER_ROS_INTERFACE_H
