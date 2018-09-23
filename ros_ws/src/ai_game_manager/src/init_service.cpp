#include <utility>

#include "ai_game_manager/init_service.h"

using namespace std;

const string READY_SRV = "/ai/game_manager/node_ready";
const string ARM_SRV   = "/ai/game_manager/arm";
const string HALT_SRV  = "/ai/game_manager/status";
const auto TIMEOUT_READY_SRV = ros::Duration(15.0);

StatusServices::StatusServices(const string& namespaceName, const string& packageName, ArmCallback_t armCallback, StatusCallback_t statusCallback) :
    _armCallback(std::move(armCallback)),
    _statusCallback(std::move(statusCallback))
{
    _nodeName = "/" + namespaceName + "/" + packageName;
    if (_armCallback)
        _armServer = _nodeHandle.subscribe(ARM_SRV, 10, &StatusServices::_on_arm, this);
    if (_statusCallback)
        _gameStatusSubscriber = _nodeHandle.subscribe(HALT_SRV, 10, &StatusServices::_on_gameStatus, this);
}

void StatusServices::setReady(bool success)
{
    try
    {
        if (!ros::service::waitForService(READY_SRV, TIMEOUT_READY_SRV))
            throw Errors::SERVICE_TIMEOUT;
        ros::ServiceClient readyPub = _nodeHandle.serviceClient<ai_game_manager::NodeReady>(READY_SRV);
        ai_game_manager::NodeReady msg;
        msg.request.success = success;
        msg.request.node_name = _nodeName;
        if (!readyPub.call(msg))
            throw Errors::SERVICE_NOT_RESPONDING;
        if (success)
            ROS_INFO_STREAM("Node " << _nodeName << " initialized successfully.");
        else
            ROS_ERROR_STREAM("Node " << _nodeName << " didn't initialize correctly.");
    }
    catch(...)
    {
        ROS_ERROR("status_services couldn't contact ai/game_manager to send init notification.");
    }
}


void StatusServices::_on_arm(const ai_game_manager::ArmRequest::ConstPtr& msg)
{
    _armCallback(msg);
}

void StatusServices::_on_gameStatus(const ai_game_manager::GameStatus::ConstPtr& msg)
{
    _statusCallback(msg);
}