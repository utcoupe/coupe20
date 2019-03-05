#include "serial.h"

#include <stdlib.h>


const size_t BUFFER_READ = 100;

Serial::Serial(UART_HandleTypeDef* serial):
    _serialInterfacePtr(serial) {
    _lastStatus = HAL_OK;
}

Serial::~Serial() {
    free(_serialInterfacePtr);
}

void Serial::print(const char* data) {
    _lastStatus = HAL_UART_Transmit(
        _serialInterfacePtr,
        (uint8_t*)data,
        strlen(data),
        _timeout
    );
}

void Serial::println(const char* data) {
    print(data);
    print("\r\n");
}

uint8_t Serial::read() {
    uint8_t rx_char;
    _lastStatus = HAL_UART_Receive(
        _serialInterfacePtr,
        &rx_char,
        1,
        _timeout
    );
    return rx_char;
}

String Serial::readStringUntil(char ch) {
    static char result[BUFFER_READ];
    char readChar;
    unsigned pos = 0;
    do {
        readChar = static_cast<char>(read());
        result[pos] = readChar;
        pos++;
    } while (_lastStatus == HAL_OK && readChar != ch && pos + 1 < BUFFER_READ);
    result[pos] = '\0';
    return result;
}

void Serial::setTimeout(uint16_t timeout) {
    _timeout = timeout;
}
