#include "pathfinder/pathfinder.h"

#include "pathfinder/dynamic_barriers_manager.h"
#include "pathfinder/occupancy_grid.h"

#include "pathfinder/curvetrajectory.h"
#include <ros/console.h>

#include <chrono>
#include <limits>
#include <sstream>

using namespace std;

Pathfinder::Pathfinder(DynamicBarriersManager &dynBarriersMng, pathfinder::OccupancyGrid &occupancyGrid, const string &mapFileName) : _dynBarriersMng(dynBarriersMng), _occupancyGrid(occupancyGrid)
{
    _renderAfterComputing = false;
    _renderFile = "tmp.bmp";

    if (!mapFileName.empty())
    {
        ROS_INFO("Loading walls from image...");
        auto allowedPositions = _mapStorage.loadAllowedPositionsFromFile(mapFileName);
        _occupancyGrid.setOccupancyFromGrid(allowedPositions);
        ROS_INFO_STREAM("Done loading, map size is " << _occupancyGrid.getSize().first << "*" << _occupancyGrid.getSize().second << "px.");
    }
    else if (_occupancyGrid.empty())
    {
        ROS_WARN("Pathfinder may not be ready, no map loaded yet.");
    }
    else
    {
        ROS_INFO_STREAM("Map already loaded, map size is " << _occupancyGrid.getSize().first << "*" << _occupancyGrid.getSize().second << "px.");
    }
}

Pathfinder::FindPathStatus Pathfinder::findPath(
    const Point &startPos,
    const Point &endPos,
    const std::vector<std::string> &ignoredTags,
    Path &path)
{
    ROS_DEBUG_STREAM("START: " << startPos);
    ROS_DEBUG_STREAM("END: " << endPos);

    auto gridSize = _occupancyGrid.getSize();

    if (gridSize.first == 0 || gridSize.second == 0)
    {
        ROS_ERROR("Allowed positions is empty. Did you load the file?");
        return FindPathStatus::MAP_NOT_LOADED;
    }

    _dynBarriersMng.fetchOccupancyDatas(gridSize.first, gridSize.second, ignoredTags);

    auto startTime = chrono::high_resolution_clock::now();

    if (!isValid(startPos) || !isValid(endPos))
    {
        ROS_ERROR("Start or end position is not valid!");
        if (_renderAfterComputing)
            _mapStorage.saveMapToFile(_renderFile, _occupancyGrid, _dynBarriersMng, Path(), Path());
        return FindPathStatus::START_END_POS_NOT_VALID;
    }

    // Creates a map filled with -1
    auto mapDist = Vect2DShort(
        gridSize.second, vector<short>(gridSize.first, -1));
    if (!exploreGraph(mapDist, startPos, endPos)) // endPos not found or no paths exist between startPos and endPos
    {
        if (_renderAfterComputing)
            _mapStorage.saveMapToFile(_renderFile, _occupancyGrid, _dynBarriersMng, {startPos}, {endPos});
        ROS_ERROR_STREAM("No path found !");
        return FindPathStatus::NO_PATH_FOUND;
    }

    Path rawPath = retrievePath(mapDist, startPos, endPos);
    path = smoothPath(rawPath);
    
    // Get Bezier curve from the straight path
    CurveTrajectory curvedPath(path);
    path = curvedPath.compute(15);

    auto endTime = chrono::high_resolution_clock::now();
    chrono::duration<double, std::milli> elapsedSeconds = endTime - startTime;
    ROS_INFO_STREAM("Found a path with " << path.size() << " points (took " << elapsedSeconds.count() << " ms)");


    if (_renderAfterComputing)
        _mapStorage.saveMapToFile(_renderFile, _occupancyGrid, _dynBarriersMng, rawPath, path);

    return FindPathStatus::NO_ERROR;
}

void Pathfinder::activatePathRendering(bool activate)
{
    _renderAfterComputing = activate;
}

void Pathfinder::setPathToRenderOutputFile(const std::string &path)
{
    _renderFile = path;
}

bool Pathfinder::exploreGraph(Vect2DShort &distMap, const Point &startPos, const Point &endPos)
{
    vector<Point> previousPositions, nextPositions;
    short distFromEnd = 0;

    distMap[endPos.getY()][endPos.getX()] = distFromEnd;
    if (startPos == endPos)
        return true;

    previousPositions.push_back(endPos);
    distFromEnd++;

    while (!previousPositions.empty())
    {
        for (const Point &prevPos : previousPositions)
        {
            for (const Point &dir : m_directions)
            {
                Point nextPos = prevPos + dir;
                if (isValid(nextPos) && distMap[nextPos.getY()][nextPos.getX()] == -1)
                {
                    distMap[nextPos.getY()][nextPos.getX()] = distFromEnd;
                    if (nextPos == startPos)
                    {
                        ROS_DEBUG("Goal Found!");
                        return true;
                    }
                    nextPositions.push_back(nextPos);
                }
            }
        }

        previousPositions.swap(nextPositions); // Swap contents not to deallocate and then reallocate memory
        nextPositions.clear();
        distFromEnd++;
    }

    return false; // if we reach this point, we haven't found start position
}

Pathfinder::Path Pathfinder::retrievePath(const Vect2DShort &distMap, const Point &startPos, const Point &endPos)
{
    Path path;
    path.push_back(startPos);

    Point lastPos = startPos;

    while (lastPos != endPos)
    {
        Point bestNextPos;
        short bestDist = numeric_limits<short>::max();
        for (const Point &dir : m_directions)
        {
            Point nextPos = lastPos + dir;
            if (isValid(nextPos))
            {
                short posDist = distMap[nextPos.getY()][nextPos.getX()];
                if (posDist >= 0 && posDist < bestDist)
                {
                    bestDist = posDist;
                    bestNextPos = nextPos;
                }
            }
        }
        lastPos = bestNextPos;
        path.push_back(bestNextPos);
    }
    return path;
}

Pathfinder::Path Pathfinder::smoothPath(const Path &rawPath)
{
    Path newPath;

    newPath.push_back(rawPath.front());
    unsigned posL = 0;
    while (posL + 1 < rawPath.size())
    {
        unsigned int posR = rawPath.size() - 1; // If size() = 0 we don't enter the loop
        for (; posR > posL + 1; posR--)
            if (canConnectWithLine(rawPath[posL], rawPath[posR]))
                break;
        posL = posR; // posR was >= posL + 1, so posL[t+1] >= posL[t] + 1
        newPath.push_back(rawPath[posL]);
    }

    return newPath;
}

bool Pathfinder::isValid(const Point &pos)
{
    if (pos.getY() < 0 || pos.getY() >= _occupancyGrid.getSize().second)
        return false;
    if (pos.getX() < 0 || pos.getX() >= _occupancyGrid.getSize().first)
        return false;
    if (!_occupancyGrid.isAllowed(pos) || _dynBarriersMng.hasBarriers(pos))
        return false;
    return true;
}

bool Pathfinder::canConnectWithLine(const Point &pA, const Point &pB)
{
    Point testPos = pA;
    int dX, dY, stepX, stepY, error;

    dX = abs(pB.getX() - pA.getX());
    dY = abs(pB.getY() - pA.getY());

    stepX = (pB.getX() > pA.getX()) ? 1 : -1;
    stepY = (pB.getY() > pA.getY()) ? 1 : -1;

    // We don't need to check start and end positions

    if (dX > dY)
    {
        error = dX / 2;
        for (int i = 0; i < dX; i++)
        {
            testPos = testPos + Point(stepX, 0);
            error += dY;
            if (error > dX)
            {
                error -= dX;
                testPos = testPos + Point(0, stepY);
            }
            if (!isValid(testPos))
                return false;
        }
    }
    else
    {
        error = dY / 2;
        for (int i = 0; i < dY; i++)
        {
            testPos = testPos + Point(0, stepY);
            error += dX;
            if (error > dY)
            {
                error -= dY;
                testPos = testPos + Point(stepX, 0);
            }
            if (!isValid(testPos))
                return false;
        }
    }
    return true;
}

const std::vector<Point> Pathfinder::m_directions = {
    Point(0, 1),
    Point(0, -1),
    Point(1, 0),
    Point(-1, 0)

    // Add other directions if needed
};

string Pathfinder::pathMapToStr(const Path &path)
{
    ostringstream os;
    string str = "[";
    for (const Point &pos : path)
        os << pos << ", ";
    str += os.str();
    if (str.length() > 2)
        str.erase(str.end() - 2, str.end());
    str += "]";
    return str;
}
