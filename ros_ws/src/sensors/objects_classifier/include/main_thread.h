#ifndef objects_classifier_MAIN_THREAD_H
#define objects_classifier_MAIN_THREAD_H

#include <ros/ros.h>

#include "map_objects.h"
#include "map_point.h"
#include "processing_thread.h"
#include "markers_publisher.h"

#include <objects_classifier/ClassifiedObjects.h>
#include <objects_classifier/ObjectsClassifierConfig.h>

#include <belt_interpreter/BeltRects.h>
#include <dynamic_reconfigure/server.h>
#include <tf2_ros/transform_listener.h>
#include <processing_lidar_objects/Obstacles.h>

#include <array>
#include <vector>

class MainThread {
public:
    MainThread(ros::NodeHandle &nh);

    ~MainThread();

    void classify_and_publish_rects(belt_interpreter::BeltRects &rects);

    void classify_and_publish_lidar_objects(processing_lidar_objects::Obstacles &obstacles);

protected:
    // minimum fraction of a rect to be in map for it to be considered static
    // not const because of dynamic reconfigure
    float MIN_MAP_FRAC = 0.35;

    ros::NodeHandle &nh_;

    objects_classifier::ClassifiedObjects classified_objects_;
    std::mutex lists_mutex_;

    ros::Publisher pub_;

    ros::Time last_rviz_rect_pub_;
    ros::Time last_rviz_lidar_pub_;

    PointArray points_;
    std::vector<std::pair<int, geometry_msgs::TransformStamped> > rects_transforms_;
    std::vector<std::unique_ptr<ProcessingThread> > threads_;

    MapObjects map_objects_;

    MarkersPublisher markers_publisher_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tl_;

    dynamic_reconfigure::Server<objects_classifier::ObjectsClassifierConfig> dyn_server_;

    void pub_loop(const ros::TimerEvent &);

    std::pair<float, float> compute_division_steps(
            const belt_interpreter::RectangleStamped &rect);

    bool fetch_transform_and_adjust_stamp(
            belt_interpreter::RectangleStamped &rect,
            geometry_msgs::TransformStamped &transform_out);

    void transform_rect(
            belt_interpreter::RectangleStamped &rect,
            geometry_msgs::TransformStamped &transform);

    void notify_threads_and_wait(int num_points);

    void reconfigure_callback(objects_classifier::ObjectsClassifierConfig &config, uint32_t level);
};


#endif //objects_classifier_MAIN_THREAD_H
