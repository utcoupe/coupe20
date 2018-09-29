#include "pathfinder/pathfinder_ros_interface.h"

using namespace std;

PathfinderROSInterface::PathfinderROSInterface(const std::string& mapFileName, std::shared_ptr<PosConvertor> convertor)
{
    dynBarriersMng_ = make_shared<DynamicBarriersManager>();
    pathfinderPtr_ = make_unique<Pathfinder>(mapFileName, dynBarriersMng_);
    convertor_ = convertor;
    
    auto mapSize = pathfinderPtr_->getMapSize();
    if (mapSize.first == 0)
        ROS_FATAL("Allowed positions empty. Cannot define a scale. Please restart the node, it may crash soon.");
    else
    {
        if (mapSize.first * mapSize.second > 200000) {
            ROS_WARN("Map image is big, the pathfinder may be very slow ! (150x100px works fine)");
        }

        convertor_->setMapSize(make_pair<double,double>(mapSize.first, mapSize.second));
        dynBarriersMng_->setConvertor(convertor_);
        dynBarriersMng_->fetchOccupancyDatas(mapSize.first, mapSize.second);
    }
}


bool PathfinderROSInterface::findPathCallback(navigation_pathfinder::FindPath::Request& req, navigation_pathfinder::FindPath::Response& rep)
{
    Pathfinder::Path path;
    
    ROS_INFO_STREAM("Received request from (" << req.posStart.x << "," << req.posStart.y << ") to (" << req.posEnd.x << ", " << req.posEnd.y << ")");
    
    auto startPos = pose2DToPoint_(req.posStart);
    auto endPos = pose2DToPoint_(req.posEnd);
    
    auto statusCode = pathfinderPtr_->findPath(startPos, endPos, path);
    switch (statusCode) {
        case Pathfinder::FindPathStatus::MAP_NOT_LOADED: // [[fallthrough]] to be uncommented if annoying warnings
        case Pathfinder::FindPathStatus::START_END_POS_NOT_VALID:
            rep.return_code = rep.START_END_POS_NOT_VALID;
            ROS_DEBUG_STREAM("Answering: Invalid posision for start or end");
            break;

        case Pathfinder::FindPathStatus::NO_PATH_FOUND:
            rep.return_code = rep.NO_PATH_FOUND;
            ROS_DEBUG_STREAM("Answering: no path found");
            break;

        case Pathfinder::FindPathStatus::NO_ERROR:
            for (const Point& pos : path)
                rep.path.push_back(pointToPose2D_(pos));
            rep.return_code = rep.PATH_FOUND;
            rep.path.front() = req.posStart;
            rep.path.back() = req.posEnd;
            ROS_DEBUG_STREAM("Answering: " << pathRosToStr_(rep.path));
            break;
    }
    
    return true;
}

void PathfinderROSInterface::reconfigureCallback(navigation_pathfinder::PathfinderNodeConfig& config, uint32_t level)
{
    ROS_INFO_STREAM ("Reconfigure request : " << config.render << " " << config.renderFile << " " << config.safetyMargin);
    pathfinderPtr_->activatePathRendering(config.render);
    pathfinderPtr_->setPathToRenderOutputFile(config.renderFile);
    // TODO detect env var and home
    dynBarriersMng_->updateSafetyMargin(config.safetyMargin);
}

void PathfinderROSInterface::addBarrierSubscriber(DynamicBarriersManager::BarriersSubscriber && subscriber)
{
    dynBarriersMng_->addBarrierSubscriber(std::move(subscriber));
}

Point PathfinderROSInterface::pose2DToPoint_(const geometry_msgs::Pose2D& pos) const
{
    auto convertedPos = convertor_->fromRosToMapPos(pair<double, double>(pos.x, pos.y));
    return Point(convertedPos.first, convertedPos.second);
}

geometry_msgs::Pose2D PathfinderROSInterface::pointToPose2D_(const Point& pos) const
{
    auto convertedPos = convertor_->fromMapToRosPos(pair<double, double>(pos.getX(), pos.getY()));
    geometry_msgs::Pose2D newPos;
    newPos.x = convertedPos.first;
    newPos.y = convertedPos.second;
    return newPos;
}


string PathfinderROSInterface::pathRosToStr_(const vector<geometry_msgs::Pose2D>& path)
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
