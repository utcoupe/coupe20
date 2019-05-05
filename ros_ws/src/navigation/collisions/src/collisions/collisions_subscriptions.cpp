#include "collisions/collisions_subscriptions.h"

#include "collisions/shapes/circle.h"
#include "collisions/shapes/rectangle.h"
#include "collisions/shapes/segment.h"
#include "collisions/obstacle_velocity.h"

#include <static_map/MapGetContainer.h>
#include <static_map/MapGetContext.h>

#include <ros/duration.h>
#include <ros/node_handle.h>

#include <cmath>
#include <string>
#include <utility>

const double      CACHE_TIME_TF2_BUFFER  = 5.0;
const std::size_t SIZE_MAX_QUEUE         = 10;

const std::string NAVIGATOR_STATUS_TOPIC = "navigation/navigator/status";
const std::string OBJECTS_TOPIC          = "recognition/objects_classifier/objects";
const std::string ASSERV_SPEED_TOPIC     = "drivers/ard_asserv/speed";
const std::string MAP_GET_CONTEXT_SERVER = "static_map/get_context";

const std::string NODE_NAME              = "collisions";
const std::string NAMESPACE_NAME         = "navigation";

const std::string MAP_TF_FRAME           = "map";
const std::string ROBOT_TF_FRAME         = "robot";
const std::string PARAM_ROBOT_TYPE       = "/robot";
const std::string DEFAULT_ROBOT_NAME     = "gr";
const double      DEFAULT_ROBOT_WIDTH    = 0.4;
const double      DEFAULT_ROBOT_HEIGHT   = 0.25;

Position quaternionToEuler(geometry_msgs::Quaternion quaternion) noexcept;

inline constexpr double radToDegrees(double angle) noexcept {
    return angle * M_PI / 180;
}

CollisionsSubscriptions::CollisionsSubscriptions(ros::NodeHandle& nhandle):
    m_obstaclesStack(std::make_shared<ObstaclesStack>()),
    m_tf2PosBuffer(ros::Duration(CACHE_TIME_TF2_BUFFER)),
    m_tf2PosListener(m_tf2PosBuffer)
{
    m_asservSpeedSubscriber = nhandle.subscribe(
        ASSERV_SPEED_TOPIC,
        SIZE_MAX_QUEUE,
        &CollisionsSubscriptions::m_onAsservSpeed,
        this
    );
    m_navigatorStatusSubscriber = nhandle.subscribe(
        NAVIGATOR_STATUS_TOPIC,
        SIZE_MAX_QUEUE,
        &CollisionsSubscriptions::m_onNavStatus,
        this
    );
    
    m_objectsSubscriber = nhandle.subscribe(
        OBJECTS_TOPIC,
        SIZE_MAX_QUEUE,
        &CollisionsSubscriptions::m_onObjects,
        this
    );
    
    m_gameStatus = std::make_unique<StatusServices>(NAMESPACE_NAME, NODE_NAME, nullptr, [this](const game_manager::GameStatus::ConstPtr& status) {
        this->m_onGameStatus(status);
    });
}

void CollisionsSubscriptions::sendInit(bool success) {
    m_gameStatus->setReady(success);
}

CollisionsSubscriptions::RobotPtr CollisionsSubscriptions::createRobot(ros::NodeHandle& nhandle)
{
    std::lock_guard<std::mutex> lock(m_mutexRobot);
    double width, height;
    auto success = false;
    try {
        ros::ServiceClient srvGetContext = nhandle.serviceClient<static_map::MapGetContext>(MAP_GET_CONTEXT_SERVER);
        
        if (!srvGetContext.exists()) {
            ROS_INFO_STREAM(
                "Waiting for service \"" << MAP_GET_CONTEXT_SERVER << "\"..."
            );
            srvGetContext.waitForExistence();
            ROS_INFO("Continuing.");
        }
        
        static_map::MapGetContext srv;
        if (!srvGetContext.call(srv)) {
            ROS_ERROR("PathfinderROSInterface::_updateMarginFromStaticMap(): Cannot contact static_map.");
            success = false;
        } else {
            const auto& shape = srv.response.robot_shape;
            switch (shape.shape_type) {
            case static_map::MapObject::SHAPE_RECT:
                width = shape.width;
                height = shape.height;
                success = true;
                break;
            case static_map::MapObject::SHAPE_CIRCLE:
                ROS_ERROR("Robot shape circle not implemented yet.");
                break;
            default:
                ROS_FATAL_ONCE("PathfinderROSInterface::_updateMarginFromStaticMap(): Unknown shape. This message will print once.");
            }
        }
    } catch(const ros::Exception& e) {
        ROS_ERROR_STREAM(
            "Error when trying to contact '" << MAP_GET_CONTEXT_SERVER << "' : "
            << e.what()
        );
    } catch(...) {
        ROS_ERROR_STREAM(
            "Unknown error when trying to contact '" << MAP_GET_CONTEXT_SERVER
            << "'."
        );
    }
    if (!success) {
        width = DEFAULT_ROBOT_WIDTH;
        height = DEFAULT_ROBOT_HEIGHT;
        ROS_WARN("Falling back to default robot's size value...");
    }
    m_robot = std::make_shared<Robot>(width, height);
    return m_robot;
}

void CollisionsSubscriptions::updateRobot()
{
    std::lock_guard<std::mutex> lock(m_mutexRobot);
    if (!m_robot)
        return;
    auto newPos = m_updateRobotPos();
    if (newPos != Position(0,0,0)) {
        m_robot->setPos(newPos);
    }
    
    m_robot->updateStatus(m_navStatus);
    
    if (!m_robotPathWaypoints.empty()) {
        m_robot->updateWaypoints(m_robotPathWaypoints);
    }
    m_robot->updateVelocity(m_velLinear, m_velAngular);
}


void CollisionsSubscriptions::m_onAsservSpeed(const ard_asserv::RobotSpeed::ConstPtr& speed)
{
    std::lock_guard<std::mutex> lock(m_mutexRobot);
    m_velLinear = static_cast<double>(speed->linear_speed);
    m_velAngular = 0.0;
}


void CollisionsSubscriptions::m_onGameStatus(const game_manager::GameStatus::ConstPtr& status) {
    // Nothing ?
}

void CollisionsSubscriptions::m_onNavStatus(const navigator::Status::ConstPtr& status)
{
    std::lock_guard<std::mutex> lock(m_mutexRobot); // Needed ?
    if (status->status == status->NAV_IDLE)
        m_navStatus = Robot::NavStatus::IDLE;
    else if (status->status == status->NAV_NAVIGATING)
        m_navStatus = Robot::NavStatus::NAVIGATING;
    
    m_robotPathWaypoints.clear();
    for (auto point: status->currentPath)
        m_robotPathWaypoints.emplace_back(point);
}

void CollisionsSubscriptions::m_onObjects(const objects_classifier::ClassifiedObjects::ConstPtr& objects)
{
    std::vector<std::shared_ptr<Obstacle>> newBelt, newLidar;
    
    for (auto rect: objects->unknown_rects) {
        if (rect.header.frame_id != MAP_TF_FRAME && rect.header.frame_id != "/" + MAP_TF_FRAME) {
            ROS_WARN_STREAM("Belt rect not in /" << MAP_TF_FRAME << " tf frame, skipping.");
            continue;
        }
        auto rectShape = std::make_unique<CollisionsShapes::Rectangle>(
            Position(rect.x, rect.y, rect.a),
            rect.w,
            rect.h
        );
        newBelt.emplace_back(
            std::make_shared<Obstacle>(std::move(rectShape))
        );
    }
    if (!newBelt.empty()) {
        m_obstaclesStack->updateBeltPoints(newBelt);
    }
    
    for (auto seg: objects->unknown_segments) {
        if (seg.header.frame_id != MAP_TF_FRAME && seg.header.frame_id != "/" + MAP_TF_FRAME) {
            ROS_WARN_STREAM("Lidar segment not in /" << MAP_TF_FRAME << " tf frame, skipping.");
            continue;
        }
        auto segShape = std::make_unique<CollisionsShapes::Segment>(
            seg.segment.first_point,
            seg.segment.last_point
        );
        newLidar.emplace_back(
            std::make_shared<Obstacle>(std::move(segShape))
        );
    }
    for (auto circ: objects->unknown_circles) {
        if (circ.header.frame_id != MAP_TF_FRAME && circ.header.frame_id != "/" + MAP_TF_FRAME) {
            ROS_WARN_STREAM("Lidar circle not in /" << MAP_TF_FRAME << " tf frame, skipping.");
            continue;
        }
        double velDist = std::hypot(circ.circle.velocity.x, circ.circle.velocity.y);
        double velAngle = std::atan2(circ.circle.velocity.y, circ.circle.velocity.x);
        auto circShape = std::make_unique<CollisionsShapes::Circle>(
            Position(circ.circle.center, velAngle),
            circ.circle.radius
        );
        auto velocity = std::make_unique<ObstacleVelocity>(
            0.0,
            0.0,
            velAngle,
            velDist
        );
        newLidar.emplace_back(
            std::make_shared<Obstacle>(std::move(circShape), std::move(velocity))
        );
    }
    if (!newLidar.empty()) {
        m_obstaclesStack->updateLidarObjects(newLidar);
    }
}

Position CollisionsSubscriptions::m_updateRobotPos()
{
    double tx, ty, rz;
    try {
        auto transform = m_tf2PosBuffer.lookupTransform(MAP_TF_FRAME, ROBOT_TF_FRAME, ros::Time());
        tx = transform.transform.translation.x;
        ty = transform.transform.translation.y;
        rz = quaternionToEuler(transform.transform.rotation).getAngle();
    } catch (tf2::TransformException &ex) {
        ROS_WARN_STREAM_THROTTLE(
                1,
                "Error when trying to get " << ROBOT_TF_FRAME << " tf from "
                << MAP_TF_FRAME << " tf: " << ex.what()
        );
    } catch (...) {
        ROS_ERROR_ONCE("An unknown exception happened when trying to update robot's position from tf. This message will be printed once.");
        tx = ty = rz = 0.0;
    }
    return { tx, ty, rz };
}


std::string CollisionsSubscriptions::fetchRobotName(ros::NodeHandle& nodeHandle)
{
    std::string m_robotname; // TODO use C++17 init in if statement
    if (nodeHandle.getParam(PARAM_ROBOT_TYPE, m_robotname))
    {
        return m_robotname;
    }
    ROS_WARN_STREAM(
        "Error when trying to get \"" << PARAM_ROBOT_TYPE
        << "\" param. Falling back to default name \"" << DEFAULT_ROBOT_NAME << "\"."
    );
    return DEFAULT_ROBOT_NAME;
}

Position quaternionToEuler(geometry_msgs::Quaternion quaternion) noexcept
{
    double qw, qx, qy, qz;
    qw = quaternion.w;
    qx = quaternion.x;
    qy = quaternion.y;
    qz = quaternion.z;
    
    double t0 = +2.0 * (qw * qx + qy * qz);
    double t1 = +1.0 - 2.0 * (qx * qx + qy * qy);
    double ex = std::atan2(t0, t1);
    
    double t2 = +2.0 * (qw * qy - qz * qx);
    if (t2 < -1.0)
        t2 = -1.0;
    else if (t2 > 1.0)
        t2 = 1.0;
    double ey = std::asin(t2);
    
    double t3 = +2.0 * (qw * qz + qx * qy);
    double t4 = +1.0 - 2.0 * (qy * qy + qz * qz);
    double ez = std::atan2(t3, t4);
    
    return {ex, ey, ez};
}
