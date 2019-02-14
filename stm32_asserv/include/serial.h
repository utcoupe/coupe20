#ifndef STM32_SERIAL_H
#define STM32_SERIAL_H

#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_dma.h"
#include "stm32f3xx_hal_uart.h"

#include <cstdint>
#include <string>

class Serial {
public:
    Serial (UART_HandleTypeDef* serial);
    ~Serial ();
    
    std::uint16_t available();
    HAL_StatusTypeDef getLastStatus();
    void print(std::string data);
    void println(std::string data);
    std::uint8_t read();
    std::string readStringUntil(char ch);
    void setTimeout(std::uint16_t timeout);
    
private:
    UART_HandleTypeDef* _serialInterfacePtr;
    std::uint16_t _timeout = 1;
    HAL_StatusTypeDef _lastStatus;
};

#endif // STM32_SERIAL_H
