#include "pathfinder/pathfinder_ros_interface.h"

using namespace std;

PathfinderROSInterface::PathfinderROSInterface(const std::string& mapFileName, std::shared_ptr<PosConvertor> convertor)
{
    dynBarriersMng_ = make_shared<DynamicBarriersManager>();
    pathfinderPtr_ = make_unique<Pathfinder>(mapFileName, dynBarriersMng_);
    convertor_ = convertor;
    
    auto mapSize = pathfinderPtr_->getMapSize();
    if (mapSize.getX() == 0)
        ROS_FATAL("Allowed positions empty. Cannot define a scale. Please restart the node, it may crash soon.");
    else
    {
        if (mapSize.getX() * mapSize.getY() > 200000) {
            ROS_WARN("Map image is big, the pathfinder may be very slow ! (150x100px works fine)");
        }

        convertor_->setMapSize(mapSize);
        dynBarriersMng_->setConvertor(convertor_);
        dynBarriersMng_->fetchOccupancyDatas(mapSize.getX(), mapSize.getY());
    }
}


bool PathfinderROSInterface::findPathCallback(pathfinder::FindPath::Request& req, pathfinder::FindPath::Response& rep)
{
    Pathfinder::Path path;
    
    ROS_INFO_STREAM("Received request from " << Point(req.posStart) << " to " << Point(req.posEnd));
    
    auto startPos = convertor_->fromRosToMapPos(req.posStart);
    auto endPos = convertor_->fromRosToMapPos(req.posEnd);
    
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
                rep.path.push_back(convertor_->fromMapToRosPos(pos).toPose2D());
            rep.return_code = rep.PATH_FOUND;
            rep.path.front() = req.posStart;
            rep.path.back() = req.posEnd;
            ROS_DEBUG_STREAM("Answering: " << pathRosToStr_(rep.path));
            break;
    }
    
    return true;
}

void PathfinderROSInterface::reconfigureCallback(pathfinder::PathfinderNodeConfig& config, uint32_t level)
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


string PathfinderROSInterface::pathRosToStr_(const vector<geometry_msgs::Pose2D>& path)
{
    ostringstream os;
    string str = "[";
    for (const geometry_msgs::Pose2D& pos : path)
        os << Point(pos) << ", ";
    str += os.str();
    if (str.length() > 2)
        str.erase(str.end()-2, str.end());
    str += "]";
    return str;
}
