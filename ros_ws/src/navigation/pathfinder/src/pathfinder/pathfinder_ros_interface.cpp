#include "pathfinder/pathfinder_ros_interface.h"

#include <static_map/MapGetContext.h>

using namespace std;

const string   MAP_LAYER_NAME   = "pathfinder";
const unsigned WARN_MAP_SIZE    = 200000; // In square pixels (pathfinder referencial)
const double   PRECISION_MARGIN = 0.05; // In meters (ROS referencial)

PathfinderROSInterface::PathfinderROSInterface(
        const string& mapFileName, std::shared_ptr<PosConvertor> convertor,
        const string& getTerrainSrvName, ros::NodeHandle& nh,
        std::pair<unsigned, unsigned> defaultScale
                                              ):
    dynBarriersMng_(make_shared<DynamicBarriersManager>()),
    convertor_(std::move(convertor)),
    _occupancyGrid(convertor_, defaultScale.second, defaultScale.first),
    pathfinder_(dynBarriersMng_, _occupancyGrid, mapFileName)
{
    _srvGetTerrain = nh.serviceClient<static_map::MapGetContext>(getTerrainSrvName);
    
    if (mapFileName.empty()) {
        if (!_updateMarginFromStaticMap()) {
            ROS_WARN("Cannot initialize margin or terrain with static_map service!");
        }
    }
    auto mapSize = _occupancyGrid.getSize();
    if (mapSize.first == 0)
        ROS_FATAL("Allowed positions empty. Cannot define a scale. Please restart the node, it may crash soon.");
    else
    {
        if (mapSize.first * mapSize.second > WARN_MAP_SIZE) {
            ROS_WARN("Map image is big, the pathfinder may be very slow ! (150x100px works fine)");
        }

        convertor_->setMapSize({ static_cast<double>(mapSize.first), static_cast<double>(mapSize.second) });
        dynBarriersMng_->setConvertor(convertor_);
        dynBarriersMng_->fetchOccupancyDatas(mapSize.first, mapSize.second);
    }
}


bool PathfinderROSInterface::findPathCallback(pathfinder::FindPath::Request& req, pathfinder::FindPath::Response& rep)
{
    Pathfinder::Path path;
    
    ROS_INFO_STREAM("Received request from " << Point(req.posStart) << " to " << Point(req.posEnd));
    
    auto startPos = convertor_->fromRosToMapPos(req.posStart);
    auto endPos = convertor_->fromRosToMapPos(req.posEnd);
    
    auto statusCode = pathfinder_.findPath(startPos, endPos, path);
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
    static bool firstCall = true; // TODO Find better approach
    ROS_INFO_STREAM ("Reconfigure request : " << config.render << " " << config.renderFile << " " << config.safetyMargin);
    pathfinder_.activatePathRendering(config.render);
    pathfinder_.setPathToRenderOutputFile(config.renderFile);
    if (!firstCall) {
        // TODO detect env var and home
        setSafetyMargin(config.safetyMargin);
    } else {
        ROS_INFO("Not updating margin since it is init call");
    }
    firstCall = false;
}

void PathfinderROSInterface::addBarrierSubscriber(DynamicBarriersManager::BarriersSubscriber && subscriber)
{
    dynBarriersMng_->addBarrierSubscriber(std::move(subscriber));
}

bool PathfinderROSInterface::setSafetyMargin(double margin, bool cascade)
{
    _safetyMargin = margin;
    dynBarriersMng_->updateSafetyMargin(margin);
    auto success = true;
    if (cascade) {
        ROS_INFO("Updating safety margin of pathfinder map.");
        success = _updateStaticMap(); // TODO what if something went wrong ?
    }
    return success;
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

bool PathfinderROSInterface::_updateStaticMap()
{
    if (_lockUpdateMap) {
        // _updateStaticMap is already updating
        return true; // or false ?
    }
    _lockUpdateMap = true;
    auto success = true;
    
    static_map::MapGetContext srv;
    if (!_srvGetTerrain.call(srv)) {
        ROS_ERROR("PathfinderROSInterface::_updateStaticMap(): Cannot contact static_map.");
        success = false;
    } else {
        _occupancyGrid.clear();
        
        for (const auto& layer: srv.response.terrain_layers) {
            if (layer.name == MAP_LAYER_NAME) {
                ROS_DEBUG_STREAM("Loading wall " << layer.name);
                _occupancyGrid.setOccupancyFromMap(layer.walls, false, _safetyMargin);
            }
        }
    }
    
    _lockUpdateMap = false;
    return success;
}

bool PathfinderROSInterface::_updateMarginFromStaticMap()
{
    if (_lockUpdateMargin) {
        // _updateMarginFromStaticMap is already updating
        return true;
    }
    _lockUpdateMargin = true;
    auto success = true;
    
    static_map::MapGetContext srv;
    if (!_srvGetTerrain.call(srv)) {
        ROS_ERROR("PathfinderROSInterface::_updateMarginFromStaticMap(): Cannot contact static_map.");
        success = false;
    } else {
        const auto& shape = srv.response.robot_shape;
        switch (shape.shape_type) {
        case static_map::MapObject::SHAPE_RECT:
            // TODO switch between central or offset point
            success = setSafetyMargin(std::hypot(shape.width / 2.0, shape.height / 2.0) + PRECISION_MARGIN);
            break;
        case static_map::MapObject::SHAPE_CIRCLE:
            success = setSafetyMargin(shape.radius + PRECISION_MARGIN);
            break;
        default:
            ROS_FATAL_ONCE("PathfinderROSInterface::_updateMarginFromStaticMap(): Unknown shape. This message will print once.");
            success = false;
        }
    }
    
    _lockUpdateMargin = false;
    return success;
}

