#include "serial.h"

#include <cstdlib>
#include <cstring>


const size_t BUFFER_READ = 220; // Size of 9 GOTO messages + 1 GOTOA

Serial::Serial(UART_HandleTypeDef* serial):
    _serialInterfacePtr(serial) {
    _lastStatus = HAL_OK;
}

Serial::~Serial() {
    free(_serialInterfacePtr);
}

int Serial::available()
{
    uint8_t rx_char;
    HAL_StatusTypeDef status = HAL_UART_Receive(
        _serialInterfacePtr,
        &rx_char,
        1,
        _timeout
    );;
    while (status == HAL_OK) {
        _lastReceivedChars.push(rx_char);
        status = HAL_UART_Receive(
            _serialInterfacePtr,
            &rx_char,
            1,
            _timeout
        );
    }
    return _lastReceivedChars.count();
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
    char readChar = 0;
    unsigned pos = 0;
    while (pos + 1 < BUFFER_READ && readChar != ch && _lastReceivedChars.count() > 0) {
        readChar = static_cast<char>(_lastReceivedChars.pop());
        result[pos] = readChar;
        pos++;
    }
    result[pos] = '\0';
    return result;
}

void Serial::setTimeout(uint16_t timeout) {
    _timeout = timeout;
}
