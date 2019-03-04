#ifndef STM32_SERIAL_H
#define STM32_SERIAL_H

#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_dma.h"
#include "stm32f3xx_hal_uart.h"

#include "String.h"

#include <stdint.h>

class Serial {
public:
    Serial (UART_HandleTypeDef* serial);
    ~Serial ();
    
    uint16_t available();
    HAL_StatusTypeDef getLastStatus();
    void print(const String& data) { print(data.c_str()); }
    void print(const char* data);
    void println(const String& data) { println(data.c_str()); }
    void println(const char* data);
    uint8_t read();
    String readStringUntil(char ch);
    void setTimeout(uint16_t timeout);
    
private:
    UART_HandleTypeDef* _serialInterfacePtr;
    uint16_t _timeout = 10; // TODO maybe higher value + define
    HAL_StatusTypeDef _lastStatus;
};

extern Serial g_serial;

#endif // STM32_SERIAL_H
