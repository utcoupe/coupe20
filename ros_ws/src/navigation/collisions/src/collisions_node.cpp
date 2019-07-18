#include "collisions/collisions_node_class.h"

#include <ros/ros.h>

using namespace std;

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "collisions");
    ros::NodeHandle nh;
    CollisionsNode mainNode(nh);

    ros::spin();

    return 0;
}
