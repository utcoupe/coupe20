#include "ros/ros.h"
#include "ax12_server.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "ax12");
    Ax12Server server("drivers/ax12", "drivers/ax12/set_param", "drivers/ax12/simple_command");
    ros::spin();
    return 0;
}
