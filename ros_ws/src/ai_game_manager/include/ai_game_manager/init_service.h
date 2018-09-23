#ifndef AI_GAME_STATUS_INIT_SERVICE_H
#define AI_GAME_STATUS_INIT_SERVICE_H

#include <ros/ros.h>

#include "ai_game_manager/GameStatus.h"
#include "ai_game_manager/ArmRequest.h"
#include "ai_game_manager/NodeReady.h"

#include <functional>

/**
 * @brief Main class of the lib `ai_game_manager`.
 * It implements most of the python script `init_service.py` does.
 */
class StatusServices
{
public:
    /** @brief Type of the callback for the arm request */
    using ArmCallback_t = std::function<void (const ai_game_manager::ArmRequest::ConstPtr &)>;
    /** @brief Type of the callback for the status event */
    using StatusCallback_t = std::function<void (const ai_game_manager::GameStatus::ConstPtr &)>;
    
    /**
     * @brief Initialize the status service and saves the callbacks for the future requests and events.
     * 
     * Creates the service `(nodename)/arm` that will be called by `ai_game_manager` main node when all the node in our network must arm.
     * It subscribes also to the topic `/ai/game_manager/status`. To avoid any compatibility problems between std C++ and boost, statusCallback is not directly connected, it will be called through _on_arm member function.
     * 
     * @param namespaceName The name of the node namespace.
     * @param packageName The name of the package or node.
     * @param armCallback Callback called when a resquest to arm the node has been received. The value returned by the callback is send as a response.
     * @param statusCallback Callback called when there is an update on the status topic.
     */
    StatusServices(const std::string& namespaceName, const std::string& packageName, ArmCallback_t armCallback = nullptr, StatusCallback_t statusCallback = nullptr);
    
    /**
     * @brief Notify the `ai/game_manager` main node that the current node tried to launch and say if it succeeded.
     * 
     * Calls the service `/ai/game_manager/node_ready` with the node name and the success boolean as parameters.
     * 
     * @param success Indicates if the node succeeded to start and initialize.
     */
    void setReady(bool success);

private:
    enum class Errors {SERVICE_TIMEOUT, SERVICE_NOT_RESPONDING};
    
    std::string _nodeName;
    ArmCallback_t _armCallback;
    StatusCallback_t _statusCallback;
    ros::NodeHandle _nodeHandle;
    ros::Subscriber _armServer;
    ros::Subscriber _gameStatusSubscriber;

    void _on_arm(const ai_game_manager::ArmRequest::ConstPtr& msg);
    void _on_gameStatus(const ai_game_manager::GameStatus::ConstPtr& msg);
};

#endif // AI_GAME_STATUS_INIT_SERVICE_H
