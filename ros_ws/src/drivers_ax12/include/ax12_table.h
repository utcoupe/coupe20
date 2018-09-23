
#ifndef DRIVERS_AX12_AX12_TABLE_H
#define DRIVERS_AX12_AX12_TABLE_H

#include <stdint.h>


namespace Ax12Table {
    /*
     * Defines the items of the register table.
     * This is specific the the AX-12A motor.
     * See http://support.robotis.com/en/techsupport_eng.htm#product/dynamixel/ax_series/dxl_ax_actuator.htm
     */

    struct Register {
        uint8_t address;
        uint8_t access;
        uint8_t size;
    };

    // access
    const uint8_t READ_WRITE = 1;
    const uint8_t READ = 0;

    // size
    const uint8_t WORD = 2;
    const uint8_t BYTE = 1;

    // table
    const Register
            // EEPROM
            MODEL_NUMBER          = {0,  READ,       WORD},
            VERSION_OF_FIRMWARE   = {2,  READ,       BYTE},
            ID                    = {3,  READ_WRITE, BYTE},
            BAUD_RATE             = {4,  READ_WRITE, BYTE},
            RETURN_DELAY_TIME     = {5,  READ_WRITE, BYTE},
            CW_ANGLE_LIMIT        = {6,  READ_WRITE, WORD},
            CCW_ANGLE_LIMIT       = {8,  READ_WRITE, WORD},
            HIGHEST_TEMPERATURE   = {11, READ_WRITE, BYTE},
            LOWEST_VOLTAGE        = {12, READ_WRITE, BYTE},
            HIGHEST_VOLTAGE       = {13, READ_WRITE, BYTE},
            MAX_TORQUE            = {14, READ_WRITE, WORD},
            STATUS_RETURN_LEVEL   = {16, READ_WRITE, BYTE},
            ALARM_LED             = {17, READ_WRITE, BYTE},
            ALARM_SHUTDOWN        = {18, READ_WRITE, BYTE},

            // RAM
            TORQUE_ENABLE         = {24, READ_WRITE, BYTE},
            LED                   = {25, READ_WRITE, BYTE},
            CW_COMPLIANCE_MARGIN  = {26, READ_WRITE, BYTE},
            CCW_COMPLIANCE_MARGIN = {27, READ_WRITE, BYTE},
            CW_COMPLIANCE_SLOPE   = {28, READ_WRITE, BYTE},
            CCW_COMPLIANCE_SLOPE  = {29, READ_WRITE, BYTE},
            GOAL_POSITION         = {30, READ_WRITE, WORD},
            MOVING_SPEED          = {32, READ_WRITE, WORD},
            TORQUE_LIMIT          = {34, READ_WRITE, WORD},
            PRESENT_POSITION      = {36, READ,       WORD},
            PRESENT_SPEED         = {38, READ,       WORD},
            PRESENT_LOAD          = {40, READ,       WORD},
            PRESENT_VOLTAGE       = {42, READ,       BYTE},
            PRESENT_TEMPERATURE   = {43, READ,       BYTE},
            REGISTERED            = {44, READ,       BYTE},
            MOVING                = {46, READ,       BYTE},
            LOCK                  = {47, READ_WRITE, BYTE},
            PUNCH                 = {48, READ_WRITE, WORD};
}


#endif //DRIVERS_AX12_AX12_TABLE_H
