#ifndef PATHFINDER_ROS_INTERFACE_H
#define PATHFINDER_ROS_INTERFACE_H

#include <ros/console.h>

#include "pathfinder/pathfinder.h"
#include "pathfinder/dynamic_barriers_manager.h"
#include "pathfinder/pos_convertor.h"
#include "pathfinder/occupancy_grid.h"

#include <geometry_msgs/Pose2D.h>
#include <ros/service_client.h>
#include <ros/node_handle.h>

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
        const std::string& mapFileName, std::shared_ptr<PosConvertor> convertor,
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
    
    void setSafetyMargin(double margin);
    
private:
    double _safetyMargin = 0.15;
    
    /**
     * The barrier subscribers manager
     */
    std::shared_ptr<DynamicBarriersManager> dynBarriersMng_;
    
    /** Convertor object between inside and outside referentials **/
    std::shared_ptr<PosConvertor> convertor_;
    
    pathfinder::OccupancyGrid _occupancyGrid;
    
    /**
     * Main algorithm
     */
    Pathfinder pathfinder_;
    
    ros::ServiceClient _srvGetTerrain;
    
    // Convertors
    /**
     * Convert the path in the outside type to a string for debugging purposes.
     * @param path The path in outside referential and type.
     * @return The path in string format.
     */
    std::string pathRosToStr_(const std::vector<geometry_msgs::Pose2D>& path);
    
    bool _updateStaticMap();
};

#endif // PATHFINDER_ROS_INTERFACE_H
