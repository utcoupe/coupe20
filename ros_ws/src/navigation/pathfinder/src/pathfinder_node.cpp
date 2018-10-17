#include <ros/ros.h>
#include <ros/package.h>
#include <dynamic_reconfigure/server.h>

#include "pathfinder/FindPath.h"
#include "pathfinder/PathfinderNodeConfig.h"

#include "game_manager/init_service.h"

#include "pathfinder/pathfinder_ros_interface.h"
#include "pathfinder/point.h"
#include "pathfinder/BarriersSubscribers/static_map_subscriber.h"
#include "pathfinder/BarriersSubscribers/objects_classifier_subscriber.h"
#include "pathfinder/pos_convertor.h"

#include <memory>

using namespace std;
using namespace Memory;
using namespace Recognition;

const string                NAMESPACE_NAME              = "navigation";
const string                NODE_NAME                   = "pathfinder";

const string                FINDPATH_SERVICE_NAME       = NAMESPACE_NAME + "/" + NODE_NAME + "/find_path";
const pair<double, double>  TABLE_SIZE                  = {3.0, 2.0}; // Scale corresponding to messages received by the node
const string                PR_MAP_FILE_NAME            = "layer_ground.bmp";
const string                GR_MAP_FILE_NAME            = "layer_pathfinder.bmp"; //"/ros_ws/src/navigation_pathfinder/def/map.bmp"; for debug purposes
const string                DEFAULT_ROBOT_NAME          = "gr";

const size_t                SIZE_MAX_QUEUE              = 10;
const double                SAFETY_MARGIN               = 0.15;
const string                MAP_GET_OBJECTS_SERVER      = "memory/map/get_objects";
const string                OBJECTS_CLASSIFIER_TOPIC    = "recognition/objects_classifier/objects";

/**
 * Constructs BarrierSubcribers
 * 
 * @param nodeHandle The node handle used by the node
 * @param topic The topic (or service) name the subscriber has to connect to.
 * @return A unique_ptr containing the subscriber. Will implicitly use std::move.
 */
template<typename T>
unique_ptr<T> constructSubscriber(ros::NodeHandle& nodeHandle, const string& topic);

/**
 * Retrieve the robot's name from the parameters
 * 
 * @param nodeHandle The node handle used by the node
 * @return The name of the robot
 */
string fetchRobotName(ros::NodeHandle& nodeHandle);

int main (int argc, char* argv[])
{
    ros::init(argc, argv, "pathfinder_node");
    
//     ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    ros::NodeHandle nodeHandle;
    
    // Select the configuration depending on param robot
    auto robotName = fetchRobotName(nodeHandle);
    string memoryMapPath = ros::package::getPath("memory_map") + "/def/occupancy/";
    string mapPath = memoryMapPath;
    if (robotName == "pr")
        mapPath += PR_MAP_FILE_NAME;
    else
        mapPath += GR_MAP_FILE_NAME;
    
    ROS_INFO_STREAM("Starting pathfinder with map \"" + mapPath + "\"...");
    
    // Partialy initialize the convertor
    auto convertor = make_shared<PosConvertor>();
    convertor->setInvertedY(true);
    convertor->setRosSize(TABLE_SIZE);
    
    PathfinderROSInterface pathfinderInterface(mapPath, convertor);
    
    // Add some obstacle sources
    auto mapSubscriber = constructSubscriber<MapSubscriber>(nodeHandle, MAP_GET_OBJECTS_SERVER);
    mapSubscriber->setConvertor(convertor);
    pathfinderInterface.addBarrierSubscriber(std::move(mapSubscriber));
    pathfinderInterface.addBarrierSubscriber(constructSubscriber<ObjectsClassifierSubscriber>(nodeHandle, OBJECTS_CLASSIFIER_TOPIC));

    ros::service::waitForService(MAP_GET_OBJECTS_SERVER, 20000);

    // Configure the main service
    ros::ServiceServer findPathServer = nodeHandle.advertiseService(FINDPATH_SERVICE_NAME, &PathfinderROSInterface::findPathCallback, &pathfinderInterface);
    
    // Configure the dynamic parameter service
    dynamic_reconfigure::Server<navigation_pathfinder::PathfinderNodeConfig> server;
    dynamic_reconfigure::Server<navigation_pathfinder::PathfinderNodeConfig>::CallbackType f;
    
    f = boost::bind(&PathfinderROSInterface::reconfigureCallback, &pathfinderInterface, _1, _2);
    server.setCallback(f);
    
    // Tell other node that we are ready
    StatusServices gameStatusSrv(NAMESPACE_NAME, NODE_NAME);
    gameStatusSrv.setReady(true);
    
    ros::spin();
    
    return 0;
}

template<typename T>
unique_ptr<T> constructSubscriber(ros::NodeHandle& nodeHandle, const string& topic)
{
    unique_ptr<T> subscriber = std::make_unique<T>(SAFETY_MARGIN);
    subscriber->subscribe(nodeHandle, SIZE_MAX_QUEUE, topic);
    return subscriber;
}


string fetchRobotName(ros::NodeHandle& nodeHandle)
{
    string robot_name; // TODO use C++17 init in if statement
    if (nodeHandle.getParam("/robot", robot_name))
    {
        return robot_name;
    }
    ROS_WARN_STREAM("Error when trying to get '/robot' param. Falling back to default name \"" << DEFAULT_ROBOT_NAME << "\".");
    return DEFAULT_ROBOT_NAME;
}
