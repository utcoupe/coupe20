#include "String.h"

#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>

#include "serial.h"

extern Serial g_serial;

const size_t SIZE_LONG32_STR = 11;
const size_t SIZE_INT16_STR = 6;

String::String() {
    _str = nullptr;
}

String::String(const char* str) {
    _str = nullptr;
    if (str != nullptr) {
        // +1 for the null terminator
        size_t len = strlen(str) + 1;
        _str = (char*) malloc( sizeof(char) * len );
        strncpy(_str, str, len);
    }
}

String::~String() {
    if (_str != nullptr) {
        free(_str);
        _str = nullptr;
    }
}

char String::back() const {
    if (length() == 0) {
        return '\0';
    } else {
        return _str[length() - 1];
    }
}

size_t String::length() const
{
    if (_str == nullptr)
        return 0;
    return strlen(_str);
}


void String::pop_back() {
    if (length() == 0) {
        return;
    }
    _str[length() - 1] = '\0'; // TODO maybe free memory ?
}

String& String::operator+=(const char* str) {
    if (str != nullptr && strlen(str) > 0) {
        size_t newLen = length() + strlen(str);
        size_t oldLen = length();
        char* newStr = (char*) malloc ((newLen + 1) * sizeof(char));
        if (oldLen > 0) {
            strcpy(newStr, _str);
            free(_str);
            _str = nullptr;
        }
        strcpy(newStr + oldLen, str);
        _str = newStr;
    }
    return *this;
}

bool String::operator== (const String& other) const {
    return ( strcmp(c_str(), other.c_str()) == 0 );
}

String String::operator+ (const char* str) const {
    String newStr(*this);
    return (newStr += str);
}

String String::operator+ (const String& str) const {
    String newStr(*this);
    return (newStr += str);
}

String & String::operator=(const char* str)
{
    if (_str != nullptr) {
        free(_str);
        _str = nullptr;
    }
    if (str != nullptr) {
        auto len = strlen(str) + 1;
        _str = (char*) malloc( len * sizeof(char));
        strncpy(_str, str, len);
    }
    return *this;
}
