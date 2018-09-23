
#include <tf/transform_datatypes.h>

#include "markers_publisher.h"

void MarkersPublisher::publish_rects(
        const std::vector<processing_belt_interpreter::RectangleStamped> &map_rects,
        const std::vector<processing_belt_interpreter::RectangleStamped> &unknown_rects) {

    static unsigned int id_counter = 0;

    visualization_msgs::Marker marker;
    marker.action = marker.ADD;
    marker.type = marker.CUBE;
    marker.color.a = 1.0;
    marker.color.b = 0.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.lifetime = ros::Duration(LIFETIME);
    marker.pose.position.z = Z_POS;
    marker.scale.z = Z_SCALE;
    marker.ns = "rects";

    const std::vector<processing_belt_interpreter::RectangleStamped> both_lists[2] = {std::move(map_rects),
                                                                                std::move(unknown_rects)};

    for (const auto &list : both_lists) {
        for (auto &rect : list) {
            marker.pose.position.x = rect.x;
            marker.pose.position.y = rect.y;
            marker.pose.orientation = tf::createQuaternionMsgFromYaw(rect.a);
            marker.scale.x = rect.w;
            marker.scale.y = rect.h;
            marker.header = rect.header;

            marker.id = id_counter++;
            pub_.publish(marker);
        }

        marker.color.r = 1.0;
        marker.color.g = 0.0;
    }
}

void MarkersPublisher::publish_circles(
        const std::vector<recognition_objects_classifier::CircleObstacleStamped> &map_circles,
        const std::vector<recognition_objects_classifier::CircleObstacleStamped> &unknown_circles) {

    static unsigned int id_counter = 0;

    visualization_msgs::Marker marker;
    marker.action = marker.ADD;
    marker.type = marker.CYLINDER;
    marker.color.a = 1.0;
    marker.color.b = 0.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.lifetime = ros::Duration(LIFETIME);
    marker.pose.position.z = Z_POS;
    marker.scale.z = Z_SCALE;
    marker.ns = "circles";

    const std::vector<recognition_objects_classifier::CircleObstacleStamped> both_lists[2] = {std::move(map_circles),
                                                                                        std::move(unknown_circles)};

    for (const auto &list : both_lists) {
        for (auto &circle : list) {

            marker.pose.position.x = circle.circle.center.x;
            marker.pose.position.y = circle.circle.center.y;
            marker.scale.x = circle.circle.true_radius * 2.0;
            marker.scale.y = circle.circle.true_radius * 2.0;
            marker.header = circle.header;

            marker.id = id_counter++;
            pub_.publish(marker);
        }

        marker.color.r = 1.0;
        marker.color.g = 0.0;
    }
}

void MarkersPublisher::publish_segments(
        const std::vector<recognition_objects_classifier::SegmentObstacleStamped> &map_segments,
        const std::vector<recognition_objects_classifier::SegmentObstacleStamped> &unknown_segments) {


    static unsigned int id_counter = 0;

    std::vector<geometry_msgs::Point> points;

    visualization_msgs::Marker marker;
    marker.action = marker.ADD;
    marker.type = marker.LINE_LIST;
    marker.color.a = 1.0;
    marker.color.b = 0.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.lifetime = ros::Duration(LIFETIME);
    marker.pose.position.z = Z_POS;
    marker.ns = "segments";

    geometry_msgs::Point point;

    const std::vector<recognition_objects_classifier::SegmentObstacleStamped> both_lists[2] = {std::move(map_segments),
                                                                                          std::move(unknown_segments)};

    for (const auto &list : both_lists) {
        points.clear();


        for (auto &segment : list) {
            point.x = segment.segment.first_point.x;
            point.y = segment.segment.first_point.y;
            points.push_back(point);

            point.x = segment.segment.last_point.x;
            point.y = segment.segment.last_point.y;
            points.push_back(point);
        }

        if (!list.empty()) {
            marker.header = list.at(0).header;
            marker.id = id_counter++;
            pub_.publish(marker);
        }

        marker.color.r = 1.0;
        marker.color.g = 0.0;
    }


}

bool MarkersPublisher::is_connected() {
    return pub_.getNumSubscribers() > 0;
}