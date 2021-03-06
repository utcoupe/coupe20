#ifndef PATHFINDER_ROS_INTERFACE_H
#define PATHFINDER_ROS_INTERFACE_H

#include <ros/console.h>

#include "pathfinder/pathfinder.h"
#include "pathfinder/dynamic_barriers_manager.h"
#include "pathfinder/occupancy_grid.h"

#include <geometry_msgs/Pose2D.h>
#include <ros/service_client.h>
#include <ros/node_handle.h>

#include <atomic>

class PosConvertor;

class PathfinderROSInterface
{
public:
    /**
     * Initialize the pathfinder main algorithm and some interface components.
     * 
     * @param mapFileName The path to the image containing the static obstacles.
     * @param convertor The convertor that will be used to convert positions between the two referencials.
     */
    PathfinderROSInterface(
        const std::string& mapFileName, PosConvertor& convertor,
        const std::string& getTerrainSrvName, ros::NodeHandle& nh,
        std::pair<unsigned, unsigned> defaultScale = {}
    );
    
    /**
     * Callback for the ros FindPath service. Coordinates are converted between the outside and inside referential.
     * @param req The request, containing the start and end positions.
     * @param rep The response, will contain the shortest path if it exists.
     */
    bool findPathCallback(pathfinder::FindPath::Request &req, pathfinder::FindPath::Response &rep);
    
    /**
     * Callback for ros PathfinderNodeConfig dynamic reconfigure service.
     * @param config The new configuration to use.
     * @param level *Not used.*
     */
    void reconfigureCallback(pathfinder::PathfinderNodeConfig &config, uint32_t level);
    
    /**
     * Adds a subscriber to the manager.
     * 
     * @param subscriber A unique_ptr&& containing the initialized subscriber.
     */
    void addBarrierSubscriber(DynamicBarriersManager::BarriersSubscriber && subscriber);
    
    bool setSafetyMargin(double margin, bool cascade = false);
    
private:
    double _safetyMargin = 0.15;
    
    /**
     * Prevents PathfinderROSInterface::_updateStaticMap to run multiple times simultanously
     */
    std::atomic<bool> _lockUpdateMap { false };
    
    /**
     * Prevents PathfinderROSInterface::_updateMarginFromStaticMap to run multiple times simultanously
     */
    std::atomic<bool> _lockUpdateMargin { false };
    
    /**
     * The barrier subscribers manager
     */
    DynamicBarriersManager dynBarriersMng_;
    
    /** Convertor object between pathfinder and ros referentials **/
    PosConvertor& convertor_;
    
    pathfinder::OccupancyGrid _occupancyGrid;
    
    /**
     * Main algorithm
     */
    Pathfinder pathfinder_;
    
    ros::ServiceClient _srvGetTerrain;
    
    /**
     * Convert the path from geometry_msgs::Pose2D type to a string for debugging purposes.
     * @param path The path in outside referential and type.
     * @return The path in string format.
     */
    std::string pathRosToStr_(const std::vector<geometry_msgs::Pose2D>& path);
    
    /**
     * Updates the pathfinder map and the safety margin
     */
    bool _updateStaticMap();
    
    /**
     * Computes the safety margin with the robot shape given by static_map.
     */
    bool _updateMarginFromStaticMap();
};

#endif // PATHFINDER_ROS_INTERFACE_H
