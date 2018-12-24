#include <ros/ros.h>

#include <static_map/MapGetContainer.h>
#include <static_map/MapObject.h>

#include "map_objects.h"


const std::string MAP_GET_SERVICE = "memory/map/get";
const std::string MAP_OBJECTS = "map";

void MapObjects::fetch_map_objects() {
    ros::ServiceClient client = nh_.serviceClient<static_map::MapGetContainer>(MAP_GET_SERVICE);

    client.waitForExistence();

    static_map::MapGetContainer srv;

    srv.request.path = MAP_OBJECTS;

    if (client.call(srv) && srv.response.success) {
        const auto& mapObjects = srv.response.container.objects;

        map_shapes_.clear();
        for (auto &mapObject : mapObjects) {
            switch(mapObject.shape_type) {
                case static_map::MapObject::SHAPE_RECT:
                    map_shapes_.push_back(std::make_shared<Rectangle>(
                        mapObject.pose.x,
                        mapObject.pose.y,
                        mapObject.width,
                        mapObject.height
                    ));
                    break;
                
                case static_map::MapObject::SHAPE_CIRCLE:
                    map_shapes_.push_back(std::make_shared<Circle>(
                        mapObject.pose.x,
                        mapObject.pose.y,
                        mapObject.radius
                    ));
                    break;
                
                case static_map::MapObject::SHAPE_POINT:
                    ROS_WARN_ONCE("[MapObjects::fetch_map_objects] Point shape not supported!");
                    break;
                
                default:
                    ROS_ERROR("[MapObjects::fetch_map_objects] Unknown shape!");
            }

        }

        ROS_INFO_STREAM("Fetched " << map_shapes_.size() << " map shapes successfully");

    } else {
        ROS_ERROR("Failed to contact static_map, static objects not fetched");
    }
}

bool MapObjects::contains_point(float x, float y) {
    // TODO: remove hardcoded values and fetch from map
    
    if (x > 3 - WALLS_MARGIN || x < WALLS_MARGIN || y < WALLS_MARGIN || y > 2 - WALLS_MARGIN)
        return true;

    for (auto &map_shape : map_shapes_) {
        if (map_shape->contains_point(x, y))
            return true;
    }

    return false;
}
