#ifndef PATHFINDER_H
#define PATHFINDER_H

#include <ros/console.h>

#include "pathfinder/FindPath.h"
#include "pathfinder/PathfinderNodeConfig.h"

#include "pathfinder/map_storage.h"
#include "pathfinder/pos_convertor.h"
#include "pathfinder/dynamic_barriers_manager.h"
#include "pathfinder/occupancy_grid.h"

#include <geometry_tools/point.h>

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
    using Path = std::vector<Point>;
    
    /**
     * Defines return statuses for the find path algorithm.
     */
    enum class FindPathStatus {NO_ERROR, NO_PATH_FOUND, MAP_NOT_LOADED, START_END_POS_NOT_VALID};

    /**
     * Initialize the pathfinder by loading given image file containing static barriers positions, and the size of a rectangle containing all input positions (here the table).
     * @param mapFileName The name of the image file to load.
     * @param dynBarriersMng The dynamic barriers manager already initialized.
     */
    Pathfinder(std::shared_ptr<DynamicBarriersManager> dynBarriersMng, pathfinder::OccupancyGrid& occupancyGrid, const std::string& mapFileName = "");
    
    /**
     * Try to find a path between the two given positions. The coordinates are directly used in inside referential. It returns false if no paths where found.
     * @param startPos Start position.
     * @param endPos End position.
     * @param path Will contain the shortest path between startPos and endPos if a path was found.
     */
    FindPathStatus findPath(const Point& startPos, const Point& endPos, Path& path);
    
    /**
     * Set to true to generate a debug image after all searches
     */
    void activatePathRendering(bool activate);
    
    /**
     * Set the path to the debug image. See activatePathRendering to activate this feature.
     * @TODO Use instead C++17 filesystem::path
     * 
     * @param path The path to the image.
     */
    void setPathToRenderOutputFile(const std::__cxx11::string& path);

private:
    /** Shortcut to define a 2D array of short. **/
    using Vect2DShort = std::vector<std::vector<short> >;
    /** Shortcut to define a 2D array of bool. vector<bool> is a special type, different from vector<T> **/
    using Vect2DBool = std::vector<std::vector<bool> >;
    
    /** Manager for loading and saving image files **/
    MapStorage _mapStorage;
    /** Contains the positions of dynamic barriers. **/
    std::shared_ptr<DynamicBarriersManager> _dynBarriersMng;
    
    /** Tells if the map and the path must be saved after computing. **/
    bool _renderAfterComputing;
    /** Name of the file that will be generated after computing. **/
    std::string _renderFile;
    
    pathfinder::OccupancyGrid& _occupancyGrid;
    
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
    static const std::vector<Point> directions();
    
    /**
     * Convert the path in the inside type to a string for debugging purposes.
     * @param path The path in inside referential and type.
     * @return The path in string format.
     */
    std::string pathMapToStr(const Path& path);
};

#endif // PATHFINDER_H
