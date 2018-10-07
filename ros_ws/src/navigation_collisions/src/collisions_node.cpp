#include <ros/ros.h>

#include <iostream>

using namespace std;

int main(int argc, char *argv[]){
    ros::init(argc, argv, "collisions_node");
    
    ros::spin();
    
    return 0;
}
