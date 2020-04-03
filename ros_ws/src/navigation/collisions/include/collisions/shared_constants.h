#ifndef COLLISIONS_SHARED_CONSTANTS_H
#define COLLISIONS_SHARED_CONSTANTS_H

namespace collisions {
    const char *const NAVIGATOR_STATUS_TOPIC = "navigation/navigator/status";
    const char *const OBJECTS_TOPIC = "recognition/objects_classifier/objects";
    const char *const ASSERV_SPEED_TOPIC = "drivers/ard_asserv/speed";
    const char *const MAP_GET_CONTEXT_SERVER = "static_map/get_context";

    const char *const NODE_NAME = "collisions";
    const char *const NAMESPACE_NAME = "navigation";

    const std::string MAP_TF_FRAME = "map";
    const char *const ROBOT_TF_FRAME = "robot";
    
    // Default robot dimensions (in meters)
    const double DEFAULT_ROBOT_WIDTH = 0.4;
    const double DEFAULT_ROBOT_HEIGHT = 0.25;
}

#endif // COLLISIONS_SHARED_CONSTANTS_H
