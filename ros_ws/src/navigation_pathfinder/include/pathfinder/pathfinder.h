#ifndef PATHFINDER_H
#define PATHFINDER_H

#include <ros/console.h>

#include "navigation_pathfinder/FindPath.h"
#include "navigation_pathfinder/PathfinderNodeConfig.h"

#include "pathfinder/map_storage.h"
#include "pathfinder/point.h"
#include "pathfinder/pos_convertor.h"
#include "pathfinder/dynamic_barriers_manager.h"

#include <vector>
#include <memory>


/**
 * Main class for the pathfinder algorithm.
 * From arrays reprenting the static and dynamic barriers, it looks for a path between two positions and returns the shortest one if at least one exists.
 */
class Pathfinder
{
public:
    /**
     * Shortcut for path in inside referential and type.
     */
    typedef std::vector<Point> Path;

    /**
     * Initialize the pathfinder by loading given image file containing static barriers positions, and the size of a rectangle containing all input positions (here the table).
     * @param mapFileName The name of the image file to load.
     * @param convertor The convertor shared with other parts of the programm.
     * @param tableSize The reference size in the outside referential.
     * @param dynBarriersMng The dynamic barriers manager already initialized.
     * @param render Tells if an image must be generated to debug if a path was found.
     * @param renderFile The image file name that will be generated to debug.
     */
    Pathfinder(const std::string& mapFileName, std::shared_ptr<PosConvertor> convertor, const std::pair< double, double >& tableSize, std::shared_ptr<DynamicBarriersManager> dynBarriersMng, bool render = false, const std::string& renderFile = "tmp.bmp");
    
    /**
     * Try to find a path between the two given positions. The coordinates are directly used in inside referential. It returns false if no paths where found.
     * @param startPos Start position.
     * @param endPos End position.
     * @param path Will contain the shortest path between startPos and endPos if a path was found.
     */
    bool findPath(const Point& startPos, const Point& endPos, Path& path);
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

private:
    /** Shortcut to define a 2D array of short. **/
    typedef std::vector<std::vector<short> > Vect2DShort;
    /** Shortcut to define a 2D array of bool. vector<bool> is a special type, different from vector<T> **/
    typedef std::vector<std::vector<bool> > Vect2DBool;
    
    /** Convertor object between inside and outside referentials **/
    std::shared_ptr<PosConvertor> _convertor;
    
    /** Manager for loading and saving image files **/
    MapStorage _mapStorage;
    
    /** Contains the positions of static barriers. **/
    Vect2DBool _allowedPositions;
    /** Contains the positions of dynamic barriers. **/
    std::shared_ptr<DynamicBarriersManager> _dynBarriersMng;
    
    /** Tells if the map and the path must be saved after computing. **/
    bool _renderAfterComputing;
    /** Name of the file that will be generated after computing. **/
    std::string _renderFile;
    
    /**
     * From the end positions tries to reach the start position by increasing step-by-step the distance. For all intermedediate points it stores its distance to count it only one time and to be able after to retrieve the shortest path. Returns true if start position is reached, false else.
     * @param distMap Used to store the distance for all seen position to the end position.
     * @param startPos The start position to reach.
     * @param endPos The end position.
     * @return Tells if a path between start and end postions exists.
     **/
    bool exploreGraph(Vect2DShort& distMap, const Point& startPos, const Point& endPos);
    /**
     * From the start position, find the shortest path to the end position by using the array containing all distances to end position. It returns the complete path from start to end position.
     * @param distMap The 2D array containing distances to end position.
     * @param startPos The start position.
     * @param endPos The end position.
     * @return The complete path between start and end positions.
     */
    Path retrievePath(const Vect2DShort& distMap, const Point& startPos, const Point& endPos);
    /**
     * Removes unnecessary positions in the path by looking for a direct line between two points without meeting any barriers. It returns the cleaned path.
     * @param rawPath The path to clean.
     * @return The cleaned path.
     */
    Path smoothPath(const Path& rawPath);
    
    /** 
     * Check if the given position is in the working referential, and if there is no barriers at the same place.
     **/
    bool isValid(const Point& pos);
    /**
     * Check by "drawing" a line between two positinos if they can be directly connected. Returns true if there is no barriers, false else.
     */
    bool canConnectWithLine(const Point& pA, const Point& pB);
    
    /**
     * Defines all possible directions to move from any positions. May be implemented as constexpression in the future.
     * @return The lists of all allowed movements.
     */
    std::vector<Point> directions() const;
    
    // Convertors
    /**
     * Converts a position from the outside referential and type to the inside ones.
     * @param pos The position in the outside referential and type.
     * @return The position in the inside referential and type.
     */
    Point pose2DToPoint(const geometry_msgs::Pose2D& pos) const;
    /**
     * Converts a position from the inside referential and type to the outside ones.
     * @param pos The position in the inside referential and type.
     * @return The position in the outside referential and type.
     */
    geometry_msgs::Pose2D pointToPose2D(const Point& pos) const;
    
    /**
     * Convert the path in the inside type to a string for debugging purposes.
     * @param path The path in inside referential and type.
     * @return The path in string format.
     */
    std::string pathMapToStr(const Path& path);
    /**
     * Convert the path in the outside type to a string for debugging purposes.
     * @param path The path in outside referential and type.
     * @return The path in string format.
     */
    std::string pathRosToStr(const std::vector<geometry_msgs::Pose2D>& path);
};

#endif // PATHFINDER_H
