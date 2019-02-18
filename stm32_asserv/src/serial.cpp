#include "serial.h"

using namespace std;

Serial::Serial(UART_HandleTypeDef* serial):
    _serialInterfacePtr(serial) {
    _lastStatus = HAL_OK;
}

Serial::~Serial() {
    delete _serialInterfacePtr;
}

std::uint16_t Serial::available() {
    return _serialInterfacePtr->RxXferSize;
}

void Serial::print(const string& data) {
    _lastStatus = HAL_UART_Transmit(
        _serialInterfacePtr,
        (uint8_t*)data.c_str(),
        data.length(),
        _timeout
    );
}

void Serial::println(const string& data) {
    print(data + string("\n"));
}

std::uint8_t Serial::read() {
    uint8_t rx_char;
    _lastStatus = HAL_UART_Receive(
        _serialInterfacePtr,
        &rx_char,
        1,
        _timeout
    );
}

string Serial::readStringUntil(char ch) {
    string result = "";
    char readChar;
    do {
        readChar = static_cast<char>(read());
        result += readChar;
    } while (_lastStatus == HAL_OK && readChar != ch);
    return result;
}

void Serial::setTimeout(std::uint16_t timeout) {
    _timeout = timeout;
}
