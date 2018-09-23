#include "ax12_driver.h"
#include <unistd.h>
#include <ros/console.h>

using namespace Ax12Table;

bool Ax12Driver::initialize(const std::string &port_name) {
    port_handler = std::unique_ptr<dynamixel::PortHandler>(
            dynamixel::PortHandler::getPortHandler(port_name.c_str()));
    packet_handler = std::unique_ptr<dynamixel::PacketHandler>(
            dynamixel::PacketHandler::getPacketHandler(1.0));

    if (!port_handler->openPort()) {
        ROS_ERROR_STREAM("SDK could not open the port '" << port_name << "'");
        return false;
    }

    int baud_rate = 2000000 / (BAUD_RATE_INDEX + 1);
    if (!port_handler->setBaudRate(baud_rate)) {
        ROS_ERROR_STREAM("SDK could not set the baud rate at " << baud_rate);
        return false;
    }

    ROS_INFO_STREAM("SDK successfully initialized on port '" << port_name
                    << "' with baud rate " << baud_rate);

    return true;
}

void Ax12Driver::scan_motors() {
    ROS_DEBUG_STREAM("Scanning motors with IDs from 1 to " << SCAN_RANGE
                    <<", pinging " << PING_PASS_NBR << " times each");

    motor_ids.clear();

    for (uint8_t id = 1; id <= SCAN_RANGE; id++) {
        if (motor_id_connected(id)) {
            ROS_INFO("Found motor with ID %d", id);
            motor_ids.push_back(id);
        }
    }

    ROS_INFO_STREAM("Found " << motor_ids.size() << " motors while scanning");
}

bool Ax12Driver::write_register(uint8_t motor_id, const Register reg, uint16_t value) {
    if (reg.access == READ) {
        ROS_WARN_STREAM("Cannot write value " << value << " to a read-only register for motor " << static_cast<unsigned>(motor_id));
        return false;
    }

    int result;

    if (reg.size == BYTE)
        result = packet_handler->write1ByteTxRx(port_handler.get(), motor_id, reg.address,
                                                static_cast<uint8_t>(value));
    else
        result = packet_handler->write2ByteTxRx(port_handler.get(), motor_id, reg.address, value);

    if (result != COMM_SUCCESS) {
        ROS_WARN_STREAM("Could not write value " << value << " to register for motor " << static_cast<unsigned>(motor_id)
                        << " : " << packet_handler->getTxRxResult(result));
        return false;
    }

    return true;
}

bool Ax12Driver::read_register(uint8_t motor_id, const Register reg, uint16_t &value) {
    int result;

    if (reg.size == BYTE) {
        uint8_t ret_val;
        result = packet_handler->read1ByteTxRx(port_handler.get(), motor_id, reg.address, &ret_val);
        value = ret_val;
    } else {
        uint16_t ret_val;
        result = packet_handler->read2ByteTxRx(port_handler.get(), motor_id, reg.address, &ret_val);
        value = ret_val;
    }

    if (result != COMM_SUCCESS) {
        ROS_WARN_STREAM("Could not read register for motor " << static_cast<unsigned>(motor_id) << " : " << packet_handler->getTxRxResult(result));
        return false;
    }

    return true;
}

bool Ax12Driver::motor_id_exists(uint8_t motor_id) {
    return std::find(motor_ids.begin(), motor_ids.end(), motor_id) != motor_ids.end();
}

bool Ax12Driver::motor_id_connected(uint8_t motor_id) {
    int result;
    uint8_t err;

    for (uint8_t j = 0; j < PING_PASS_NBR; j++) {
        result = packet_handler->ping(port_handler.get(), motor_id, &err);

        if (err != 0) {
            ROS_DEBUG_STREAM("Could not ping motor " << static_cast<unsigned>(motor_id) << " because of an hardware error : " << packet_handler->getRxPacketError(err));
        }

        if (result == COMM_SUCCESS) {
            return true;
        }

        usleep(PING_SLEEP);
    }

    return false;
}

bool Ax12Driver::toggle_torque(bool enable) {
    ROS_DEBUG_STREAM("Toggling torque " << (enable ? "on" : "off") << " for all motors");
    bool success = true;

    for (uint8_t i : motor_ids) {
        success &= write_register(i, TORQUE_ENABLE, static_cast<uint16_t>(enable));
    }

    return success;
}

bool Ax12Driver::joint_mode(uint8_t motor_id, uint16_t min_angle, uint16_t max_angle) {
    bool success = true;

    success &= write_register(motor_id, TORQUE_ENABLE, 0);
    success &= write_register(motor_id, CW_ANGLE_LIMIT, min_angle);
    success &= write_register(motor_id, CCW_ANGLE_LIMIT, max_angle);
    success &= write_register(motor_id, TORQUE_ENABLE, 1);

    return success;
}

bool Ax12Driver::wheel_mode(uint8_t motor_id) {
    return joint_mode(motor_id, 0, 0); // wheel mode is set with min = max = 0
}
