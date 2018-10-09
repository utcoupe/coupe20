
#ifndef DRIVERS_AX12_AX12_DRIVER_H
#define DRIVERS_AX12_AX12_DRIVER_H

#include <dynamixel_sdk/dynamixel_sdk.h>
#include <memory>
#include "ax12_table.h"

class Ax12Driver {
public:

    const static uint8_t SCAN_RANGE = 10; // the scan pings motors with id 1 to SCAN_RANGE
    const static uint8_t PING_PASS_NBR = 12; // number of times a motor is pinged to make sure it is connected
    const static uint8_t BAUD_RATE_INDEX = 1; // baudrate = 2000000 / (index + 1)
    const static uint16_t PING_SLEEP = 150; // microsec to sleep between pings

protected:
    std::vector <uint8_t> motor_ids;
    std::unique_ptr <dynamixel::PortHandler> port_handler;
    std::unique_ptr <dynamixel::PacketHandler> packet_handler;

public:
    bool initialize(const std::string &port_index);

    void scan_motors();

    bool write_register(uint8_t motor_id, Ax12Table::Register reg, uint16_t value);

    bool read_register(uint8_t motor_id, Ax12Table::Register reg, uint16_t &value);

    bool motor_id_exists(uint8_t motor_id);

    bool motor_id_connected(uint8_t motor_id);

    bool toggle_torque(bool enable);

    bool joint_mode(uint8_t motor_id, uint16_t min_angle = 1, uint16_t max_angle = 1023);

    bool wheel_mode(uint8_t motor_id);

    const std::vector <uint8_t> &get_motor_ids() { return motor_ids; };

    Ax12Driver() {}

};

#endif //DRIVERS_AX12_AX12_DRIVER_H
