
#ifndef RECOGNITION_OBJECTS_CLASSIFIER_PROCESSING_THREAD_H
#define RECOGNITION_OBJECTS_CLASSIFIER_PROCESSING_THREAD_H

#include <thread>
#include <mutex>
#include <condition_variable>

#include <geometry_msgs/TransformStamped.h>

#include "map_objects.h"

struct Point;

class ProcessingThread {
protected:
    std::thread processing_thread_;
    bool thread_stopped_;

    unsigned int start_idx_;
    unsigned int length_;
    Point *points_;
    std::vector<std::pair<int, geometry_msgs::TransformStamped> > &transforms_;
    MapObjects &map_;

    // to pause the thread
    std::mutex mutex_;
    std::condition_variable cv_;

    // informs the processing_thread to start processing
    bool ready_;

    // informs the main thread that the processing is finished
    bool processed_;

    void classify_points();

public:
    void start();

    void stop();

    void join();

    void notify(unsigned int start_idx, unsigned int length);

    void wait_processing();

    ProcessingThread(Point *points, std::vector<std::pair<int, geometry_msgs::TransformStamped> > &transforms,
                     MapObjects &map) :
            points_(points),
            transforms_(transforms),
            thread_stopped_(false),
            ready_(false),
            processed_(false),
            map_(map) {}

    ProcessingThread(const ProcessingThread &other) :
            points_(other.points_),
            transforms_(other.transforms_),
            thread_stopped_(other.thread_stopped_),
            ready_(other.ready_),
            processed_(other.processed_),
            map_(other.map_) {}
};

#endif //RECOGNITION_OBJECTS_CLASSIFIER_PROCESSING_THREAD_H
