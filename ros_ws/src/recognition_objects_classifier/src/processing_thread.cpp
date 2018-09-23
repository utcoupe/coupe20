#include <iostream>
#include <geometry_msgs/PointStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>


#include "processing_thread.h"
#include "main_thread.h"

void ProcessingThread::start() {
    processing_thread_ = std::thread(&ProcessingThread::classify_points, this);
}

void ProcessingThread::classify_points() {

    geometry_msgs::PointStamped point_static_frame, point_map_frame;
    int rect_idx;
    int point_idx;
    int min_idx;

    while (!thread_stopped_) {
        // wait until main thread notification
        std::unique_lock<std::mutex> lk(mutex_);
        cv_.wait(lk, [this] { return ready_; });

        // notif received
        ready_ = false;
        processed_ = false;

        point_idx = start_idx_;

        for (auto &pair : transforms_) {
            if (pair.first < start_idx_)
                continue;

            min_idx = pair.first < start_idx_ + length_ ? pair.first : start_idx_ + length_;

            for (; point_idx <= min_idx; point_idx++) {

                point_static_frame.point.x = points_[point_idx].x;
                point_static_frame.point.y = points_[point_idx].y;
                tf2::doTransform(point_static_frame, point_map_frame, pair.second);

                points_[point_idx].is_map = map_.contains_point(point_map_frame.point.x, point_map_frame.point.y);
            }

            if (pair.first >= start_idx_ + length_)
                break;

        }

        // unlocks guard and notifies main thread we finished
        processed_ = true;
        lk.unlock();
        cv_.notify_one();
    }
}

void ProcessingThread::join() {
    processing_thread_.join();
}

void ProcessingThread::notify(unsigned int start_idx, unsigned int length) {
    // changes condition variable and notify so the thread can wake up
    {
        std::lock_guard<std::mutex> lk(mutex_);
        start_idx_ = start_idx;
        length_ = length;
        ready_ = true;
        cv_.notify_one();
    }
}


void ProcessingThread::wait_processing() {
    std::unique_lock<std::mutex> lk(mutex_);
    cv_.wait(lk, [this] { return processed_; });
}

void ProcessingThread::stop() {
    thread_stopped_ = true;
    notify(0, 0);
}