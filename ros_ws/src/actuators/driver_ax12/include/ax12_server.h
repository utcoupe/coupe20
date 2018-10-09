
#ifndef driver_ax12_AX12_SERVER_H
#define driver_ax12_AX12_SERVER_H

#include <string>
#include <list>
#include <thread>

#include <ros/console.h>
#include <actionlib/server/action_server.h>
#include <driver_ax12/Ax12CommandAction.h>
#include <driver_ax12/SetAx12Param.h>
#include <port_finder/GetPort.h>
#include <definitions/GetDefinition.h>
#include <game_manager/init_service.h>
#include <game_manager/GameStatus.h>

#include "ax12_driver.h"


const double MAX_STOP_TIME = 3; // timeout (secs)
const double MAIN_FREQUENCY = 15;
const uint8_t POSITION_MARGIN = 6;
const std::string PORT_FINDER_SERVICE = "drivers/port_finder/get_port";
const std::string GAME_STATUS_TOPIC = "ai/game_manager/status";


typedef actionlib::ServerGoalHandle <driver_ax12::Ax12CommandAction> GoalHandle;


class Ax12Server {

protected:
    ros::NodeHandle nh_;
    actionlib::ActionServer <driver_ax12::Ax12CommandAction> as_;
    ros::ServiceServer set_param_service;
    ros::Subscriber game_status_sub_;
    std::list <GoalHandle> joint_goals_;

    // create messages that are used to published feedback/result
    driver_ax12::Ax12CommandFeedback feedback_;
    driver_ax12::Ax12CommandResult result_;

    Ax12Driver driver_;
    ros::Timer timer_;

    std::unique_ptr <StatusServices> status_services_;

    bool is_halted;

public:
    void execute_goal_cb(GoalHandle goal_handle);

    void cancel_goal_cb(GoalHandle goal_handle);

    bool execute_set_service_cb(driver_ax12::SetAx12Param::Request &req, driver_ax12::SetAx12Param::Response &res);

    void game_status_cb(const game_manager::GameStatusConstPtr &status);

    std::string fetch_port(const std::string &service_name);

    void init_driver(const std::string &port);

    void main_loop(const ros::TimerEvent &);

    bool handle_joint_goal(GoalHandle goal_handle);

    bool handle_wheel_goal(GoalHandle goal_handle);

    const Ax12Table::Register *service_param_to_register(uint8_t param);

    Ax12Server(const std::string &action_name, const std::string &service_name);

    ~Ax12Server();
};

#endif //driver_ax12_AX12_SERVER_H
