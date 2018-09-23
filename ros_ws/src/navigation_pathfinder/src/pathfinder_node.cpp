#include <ros/ros.h>
#include <ros/package.h>
#include <dynamic_reconfigure/server.h>

#include "navigation_pathfinder/FindPath.h"
#include "navigation_pathfinder/PathfinderNodeConfig.h"

#include "ai_game_manager/init_service.h"

#include "pathfinder/map_storage.h"
#include "pathfinder/pathfinder.h"
#include "pathfinder/point.h"
#include "pathfinder/BarriersSubscribers/memory_map_subscriber.h"
#include "pathfinder/BarriersSubscribers/recognition_objects_classifier_subscriber.h"
#include "pathfinder/pos_convertor.h"

#include <memory>

using namespace std;
using namespace Memory;
using namespace Recognition;

const string                NAMESPACE_NAME              = "navigation";
const string                NODE_NAME                   = "pathfinder";

const string                FINDPATH_SERVICE_NAME       = "/" + NAMESPACE_NAME + "/" + NODE_NAME + "/find_path";
const pair<double, double>  TABLE_SIZE                  = {3.0, 2.0}; // Scale corresponding to messages received by the node
const string                PR_MAP_FILE_NAME            = "layer_ground.bmp";
const string                GR_MAP_FILE_NAME            = "layer_pathfinder.bmp"; //"/ros_ws/src/navigation_pathfinder/def/map.bmp"; for debug purposes

const size_t                SIZE_MAX_QUEUE              = 10;
const double                SAFETY_MARGIN               = 0.15;
const string                MAP_GET_OBJECTS_SERVER      = "/memory/map/get_objects";
const string                OBJECTS_CLASSIFIER_TOPIC    = "/recognition/objects_classifier/objects";

template<typename T>
unique_ptr<T> constructSubscriber(ros::NodeHandle& nodeHandle, const string& topic);

string fetchRobotName(ros::NodeHandle& nodeHandle);

int main (int argc, char* argv[])
{
    ros::init(argc, argv, "pathfinder_node");
    
//     ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    ros::NodeHandle nodeHandle;
    
    auto robotName = fetchRobotName(nodeHandle);
    string memoryMapPath = ros::package::getPath("memory_map") + "/def/occupancy/";
    string mapPath = memoryMapPath + GR_MAP_FILE_NAME;
    if (robotName == "pr")
        mapPath = PR_MAP_FILE_NAME;
    
    ROS_INFO_STREAM("Starting pathfinder with map \"" + mapPath + "\"...");
    
    auto convertor = make_shared<PosConvertor>();
    convertor->setInvertedY(true);
    
    auto dynBarriersMng = make_shared<DynamicBarriersManager>();
    auto mapSubscriber = constructSubscriber<MapSubscriber>(nodeHandle, MAP_GET_OBJECTS_SERVER);
    mapSubscriber->setConvertor(convertor);
    dynBarriersMng->addBarrierSubscriber(std::move(mapSubscriber));
    dynBarriersMng->addBarrierSubscriber(constructSubscriber<ObjectsClassifierSubscriber>(nodeHandle, OBJECTS_CLASSIFIER_TOPIC));

    ros::service::waitForService(MAP_GET_OBJECTS_SERVER, 20000);

    Pathfinder pathfinder(mapPath, convertor, TABLE_SIZE, dynBarriersMng);
    ros::ServiceServer findPathServer = nodeHandle.advertiseService(FINDPATH_SERVICE_NAME, &Pathfinder::findPathCallback, &pathfinder);
    
    dynamic_reconfigure::Server<navigation_pathfinder::PathfinderNodeConfig> server;
    dynamic_reconfigure::Server<navigation_pathfinder::PathfinderNodeConfig>::CallbackType f;
    
    f = boost::bind(&Pathfinder::reconfigureCallback, &pathfinder, _1, _2);
    server.setCallback(f);
    
    StatusServices gameStatusSrv(NAMESPACE_NAME, NODE_NAME);
    gameStatusSrv.setReady(true);
    
    ros::spin();
    
    return 0;
}

template<typename T>
unique_ptr<T> constructSubscriber(ros::NodeHandle& nodeHandle, const string& topic)
{
    unique_ptr<T> subscriber(new T(SAFETY_MARGIN));
    subscriber->subscribe(nodeHandle, SIZE_MAX_QUEUE, topic);
    return subscriber;
}


string fetchRobotName(ros::NodeHandle& nodeHandle)
{
    string robot_name; // TODO use C++17 init in if statement
    if (!nodeHandle.getParam("/robot", robot_name))
    {
        ROS_WARN("Error when trying to get '/robot' param. Falling back to default name \"gr\".");
        return "gr";
    }
    return robot_name;
}
