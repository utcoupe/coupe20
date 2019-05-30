#ifndef COLLISIONS_COLLISIONS_SUBSCRIPTIONS
#define COLLISIONS_COLLISIONS_SUBSCRIPTIONS

#include "collisions/robot.h"
#include "collisions/obstacles_stack.h"


#include <game_manager/init_service.h>
#include <geometry_tools/position.h>
#include <ard_asserv/RobotSpeed.h>
#include <navigator/Status.h>
#include <objects_classifier/ClassifiedObjects.h>
#include <ros/subscriber.h>
#include <tf2/buffer_core.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <mutex>
#include <vector>

/**
 * Manages all subscriptions to topics or services from other nodes.
 */
class CollisionsSubscriptions {
public:
    /** Alias for shared pointer on Robot object */
    using RobotPtr = std::shared_ptr<Robot>;
    /** Alias for shared pointer on ObstaclesStack object */
    using ObstaclesStackPtr = std::shared_ptr<ObstaclesStack>;
    /**
     * Initialize all subscribers and client services.
     * @param nhandle A valid ros::NodeHandle that will be used for all subscriptions.
     */
    CollisionsSubscriptions(ros::NodeHandle& nhandle);
    
    /**
     * Tells to the game_manager node (through the service provided by the game_manager package)
     * if the initialization succeded.
     * @param success Tells if the node is correctly started.
     */
    void sendInit(bool success = true);
    
    /**
     * Create a pointer on a Robot object that will be shared between the subscription class
     * and the main class.
     * @param nhandle A valid ros::NodeHandle
     * @return A shared pointer on the newly-created Robot
     */
    RobotPtr createRobot(ros::NodeHandle& nhandle);
    /**
     * @return A shared pointer on the ObstaclesStack object regularly updated
     */
    ObstaclesStackPtr getObstaclesStack() const noexcept { return m_obstaclesStack; }
    
    /**
     * Updates the Robot shared pointer with the last received datas.
     */
    void updateRobot();
    
private:
    /** A pointer on a Robot */
    RobotPtr m_robot;
    /** A pointer on a ObstacleStack object */
    ObstaclesStackPtr m_obstaclesStack;
    /** Ensure that the robot is not updated while being used */
    mutable std::mutex m_mutexRobot;
    
    /** Contains raw, received transform positions */
    tf2::BufferCore m_tf2PosBuffer;
    /** Listener for the tf "robot" in "map" */
    tf2_ros::TransformListener m_tf2PosListener;
    
    /** Last knonw status of the robot, received from the navigator node. */
    Robot::NavStatus m_navStatus = Robot::NavStatus::IDLE;
    /** Last knonw path of the robot, received from the navigator node. */
    std::vector<Position> m_robotPathWaypoints;
    /** Last knonw linear velocity of the robot, received from the asserv node. */
    double m_velLinear = 0.0;
    /** Last knonw angular velocity of the robot, received from the asserv node. */
    double m_velAngular = 0.0;
    
    /** Subscriber on the navigator/status topic. */
    ros::Subscriber m_navigatorStatusSubscriber;
    /** Subscriber on the objects_classifier/objects topic. */
    ros::Subscriber m_objectsSubscriber;
    /** Subscriber on the asserv/speed topic. */
    ros::Subscriber m_asservSpeedSubscriber;
    
    /** Pointer on a game_manager::GameStatus object */
    std::unique_ptr<StatusServices> m_gameStatus;
    
    /** 
     * Callback on the asserv/speed topic
     * @param speed The actualized speed of the robot
     */
    void m_onAsservSpeed(const ard_asserv::RobotSpeed::ConstPtr& speed);
    /** 
     * Callback for game_manager events
     * @TODO Useless ?
     * @param status The actualized status of the system
     */
    void m_onGameStatus(const game_manager::GameStatus::ConstPtr& status);
    /** 
     * Callback on the navigator/status topic
     * @param status Contains the status of the robot and its path
     */
    void m_onNavStatus(const navigator::Status::ConstPtr& status);
    /** 
     * Callback on the objects_classifier/objects topic
     * @param objects All obstacles detected and filtered arround the robot.
     */
    void m_onObjects(const objects_classifier::ClassifiedObjects::ConstPtr& objects);
    /**
     * Retrieves the lastest known position of the robot using tf.
     * @return Updated robot position
     */
    Position m_updateRobotPos();
};

#endif // COLLISIONS_COLLISIONS_SUBSCRIPTIONS
