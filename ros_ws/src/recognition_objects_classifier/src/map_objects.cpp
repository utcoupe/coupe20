#include <ros/ros.h>
#include <nlohmann/json.hpp>

#include <memory_map/MapGet.h>

#include "map_objects.h"

using namespace nlohmann;

void MapObjects::fetch_map_objects() {
    ros::ServiceClient client = nh_.serviceClient<memory_map::MapGet>(MAP_GET_SERVICE);

    client.waitForExistence();

    memory_map::MapGet srv;

    srv.request.request_path = MAP_OBJECTS;

    if (client.call(srv) && srv.response.success) {
        auto objects = json::parse(srv.response.response);

        map_shapes_.clear();
        for (auto &object : objects) {

            if (object["position"]["frame_id"] != "/map") {
                ROS_ERROR("Map object not in /map !");
                continue;
            }


            std::string type = object["shape"]["type"];

            float x = object["position"]["x"];
            float y = object["position"]["y"];

            if (type == "rect") {
                float height = object["shape"]["height"];
                float width = object["shape"]["width"];

                map_shapes_.push_back(std::make_shared<const Rectangle>(x, y, width, height));

            } else if (type == "circle") {
                float radius = object["shape"]["radius"];

                map_shapes_.push_back(std::make_shared<const Circle>(x, y, radius));

            } else {
                ROS_ERROR("Polygons from map not supported !");
            }

        }

        ROS_INFO_STREAM("Fetched " << map_shapes_.size() << " map shapes successfully");

    } else {
        ROS_ERROR("Failed to contact memory_map, static objects not fetched");
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
