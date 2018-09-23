#include "pathfinder/pathfinder.h"

#include <chrono>
#include <sstream>
#include <utility>
#include <limits>

using namespace std;

Pathfinder::Pathfinder(const string& mapFileName, shared_ptr<PosConvertor> convertor, const std::pair< double, double >& tableSize,
                       shared_ptr<DynamicBarriersManager> dynBarriersMng, bool render, const string& renderFile)
{
    _renderAfterComputing = render;
    _renderFile = renderFile;
    _dynBarriersMng = dynBarriersMng;
    
    _allowedPositions = _mapStorage.loadAllowedPositionsFromFile(mapFileName);
    if (_allowedPositions.size() == 0)
        ROS_FATAL("Allowed positions empty. Cannot define a scale. Please restart the node, it may crash soon.");
    else
    {
        _convertor = convertor;

        if (_allowedPositions.front().size() * _allowedPositions.size() > 200000) {
            ROS_WARN("Map image is big, the pathfinder may be very slow ! (150x100px works fine)");
        }

        _convertor->setSizes(tableSize, make_pair<double,double>(_allowedPositions.front().size(), _allowedPositions.size()));
        _dynBarriersMng->setConvertor(_convertor);
        _dynBarriersMng->fetchOccupancyDatas(_allowedPositions.front().size(), _allowedPositions.size());
    }
}


bool Pathfinder::findPath(const Point& startPos, const Point& endPos, Path& path)
{
    ROS_DEBUG_STREAM("START: " << startPos);
    ROS_DEBUG_STREAM("END: " << endPos);

    if (_allowedPositions.empty() || _allowedPositions.front().empty())
    {
        ROS_ERROR("Allowed positions is empty. Did you load the file?");
        return false;
    }

    _dynBarriersMng->fetchOccupancyDatas(_allowedPositions.front().size(), _allowedPositions.size());
    
    auto startTime = chrono::high_resolution_clock::now();
    
    // Creates a map filled with -1
    auto mapDist = Vect2DShort(
        _allowedPositions.size(), vector<short>(_allowedPositions.front().size(), -1)
    );
    if (!exploreGraph(mapDist, startPos, endPos)) // endPos not found or no paths exist between startPos and endPos
    {
        if (_renderAfterComputing)
            _mapStorage.saveMapToFile(_renderFile, _allowedPositions, _dynBarriersMng, Path(), Path());
        ROS_ERROR_STREAM("No path found !");
        return true;
    }
    
    Path rawPath = retrievePath(mapDist, startPos, endPos);
    path = smoothPath(rawPath);
    
    auto endTime = chrono::high_resolution_clock::now();
    chrono::duration<double, std::milli> elapsedSeconds = endTime - startTime;

    ROS_INFO_STREAM("Found a path with " << path.size() << " points (took " << elapsedSeconds.count() << " ms)");
    
    if (_renderAfterComputing)
        _mapStorage.saveMapToFile(_renderFile, _allowedPositions, _dynBarriersMng, rawPath, path);
    
    return true;
}

bool Pathfinder::findPathCallback(navigation_pathfinder::FindPath::Request& req, navigation_pathfinder::FindPath::Response& rep)
{
    Path path;
    
    ROS_INFO_STREAM("Received request from (" << req.posStart.x << "," << req.posStart.y << ") to (" << req.posEnd.x << ", " << req.posEnd.y << ")");
    
    auto startPos = pose2DToPoint(req.posStart);
    auto endPos = pose2DToPoint(req.posEnd);
    
    bool no_error = findPath(startPos, endPos, path);
    if (!no_error)
        return false;
    for (const Point& pos : path)
        rep.path.push_back(pointToPose2D(pos));
    if (!path.empty())
    {
        rep.success = true;
        rep.path.front() = req.posStart;
        rep.path.back() = req.posEnd;
    }
    else
        rep.success = false;
    
    ROS_DEBUG_STREAM("Answering: " << pathRosToStr(rep.path) << ", " << (rep.success?"true":"false"));
    
    return true;
}

void Pathfinder::reconfigureCallback(navigation_pathfinder::PathfinderNodeConfig& config, uint32_t level)
{
    ROS_INFO_STREAM ("Reconfigure request : " << config.render << " " << config.renderFile << " " << config.safetyMargin);
    _renderAfterComputing = config.render;
    _renderFile = config.renderFile;
    // TODO detect env var and home
    _dynBarriersMng->updateSafetyMargin(config.safetyMargin);
}



bool Pathfinder::exploreGraph(Vect2DShort& distMap, const Point& startPos, const Point& endPos)
{
    vector<Point> previousPositions, nextPositions;
    short distFromEnd = 0;
    
    if (!isValid(startPos) || !isValid(endPos))
    {
        ROS_ERROR("Start or end position is not valid!");
        return false;
    }
    
    previousPositions.push_back(endPos);
    distMap[endPos.getY()][endPos.getX()] = distFromEnd;
    distFromEnd++;
    while (!previousPositions.empty())
    {
        for (const Point& prevPos : previousPositions)
        {
            for (const Point& dir : directions())
            {
                Point nextPos = prevPos + dir;
                if (isValid(nextPos) && distMap[nextPos.getY()][nextPos.getX()] == -1)
                {
                    distMap[nextPos.getY()][nextPos.getX()] = distFromEnd;
                    if (nextPos == startPos)
                        return true;
                    nextPositions.push_back(nextPos);
                }
            }
        }
        
        previousPositions = std::move(nextPositions); // prevents use of temporary copy and nextPosition is now in undefined state
        nextPositions.clear(); // Needed to make sure it is empty
        distFromEnd++;
    }
    
    return false; // if we reach this point, we haven't found start position
}

Pathfinder::Path Pathfinder::retrievePath(const Vect2DShort& distMap, const Point& startPos, const Point& endPos)
{
    Path path;
    path.push_back(startPos);
    
    Point lastPos = startPos;
    
    while (lastPos != endPos)
    {
        Point bestNextPos;
        short bestDist = numeric_limits<short>::max();
        for (const Point& dir : directions())
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

Pathfinder::Path Pathfinder::smoothPath(const Path& rawPath)
{
    Path newPath;
    
    newPath.push_back(rawPath.front());
    unsigned posL = 0;
    while (posL + 1 < rawPath.size())
    {
        unsigned int posR = rawPath.size() - 1; // If size() = 0 we don't enter the loop
        for (;posR > posL + 1; posR--)
            if (canConnectWithLine(rawPath[posL], rawPath[posR]))
                break;
        posL = posR; // posR was >= posL + 1, so posL[t+1] >= posL[t] + 1
        newPath.push_back(rawPath[posL]);
    }
    
    return newPath;
}


bool Pathfinder::isValid(const Point& pos)
{
    if (pos.getY() < 0 || pos.getY() >= _allowedPositions.size())
        return false;
    if (pos.getX() < 0 || pos.getX() >= _allowedPositions.front().size())
        return false;
    if (!_allowedPositions[pos.getY()][pos.getX()] || _dynBarriersMng->hasBarriers(pos))
        return false;
    return true;
}

bool Pathfinder::canConnectWithLine(const Point& pA, const Point& pB)
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
        error = dX/2;
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
        error = dY/2;
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


std::vector< Point > Pathfinder::directions() const
{
    const vector<Point> dirs {
        Point(0, 1),
        Point(0, -1),
        Point(1, 0),
        Point(-1, 0)
        // Add other directions if needed
    };
    return dirs; // Should use move semantics with recent compilators
}

Point Pathfinder::pose2DToPoint(const geometry_msgs::Pose2D& pos) const
{
    auto convertedPos = _convertor->fromRosToMapPos(pair<double, double>(pos.x, pos.y));
    return Point(convertedPos.first, convertedPos.second);
}

geometry_msgs::Pose2D Pathfinder::pointToPose2D(const Point& pos) const
{
    auto convertedPos = _convertor->fromMapToRosPos(pair<double, double>(pos.getX(), pos.getY()));
    geometry_msgs::Pose2D newPos;
    newPos.x = convertedPos.first;
    newPos.y = convertedPos.second;
    return newPos;
}

string Pathfinder::pathMapToStr(const Path& path)
{
    ostringstream os;
    string str = "[";
    for (const Point& pos : path)
        os << pos << ", ";
    str += os.str();
    if (str.length() > 2)
        str.erase(str.end()-2, str.end());
    str += "]";
    return str;
}

string Pathfinder::pathRosToStr(const vector<geometry_msgs::Pose2D>& path)
{
    ostringstream os;
    string str = "[";
    for (const geometry_msgs::Pose2D& pos : path)
        os << "(" << pos.x << ", " << pos.y << ")" << ", ";
    str += os.str();
    if (str.length() > 2)
        str.erase(str.end()-2, str.end());
    str += "]";
    return str;
}
