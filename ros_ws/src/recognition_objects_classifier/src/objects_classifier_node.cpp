#include <ros/ros.h>
#include <ai_game_manager/init_service.h>

#include "main_thread.h"
#include "objects_listener.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "objects_classifier");

    ros::NodeHandle node_handle;

    MainThread mt(node_handle);
    ObjectsListener ol(node_handle, mt);

    StatusServices("recognition", "objects_classifier").setReady(true);

    ros::spin();
    return 0;
}
