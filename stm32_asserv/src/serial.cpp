#include "serial.h"

#include <stdlib.h>


Serial::Serial(UART_HandleTypeDef* serial):
    _serialInterfacePtr(serial) {
    _lastStatus = HAL_OK;
}

Serial::~Serial() {
    free(_serialInterfacePtr);
}

short unsigned int Serial::available()
{
    return _serialInterfacePtr->RxXferSize;
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
    String result = "";
    char readChar;
    do {
        readChar = static_cast<char>(read());
        result += readChar;
    } while (_lastStatus == HAL_OK && readChar != ch);
    return result;
}

void Serial::setTimeout(uint16_t timeout) {
    _timeout = timeout;
}
