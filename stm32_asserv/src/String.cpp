#include "String.h"

#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>


const size_t SIZE_LONG32_STR = 11;
const size_t SIZE_INT16_STR = 6;

String::String() {
    _str = nullptr;
}

String::String(const char* str) {
    free(_str);
    if (str != nullptr) {
        // +1 for the null terminator
        _str = (char*) malloc( sizeof(char) * (strlen(str) + 1) );
        strcpy(_str, str);
    }
}

String::String(const String& str):
    String( str.c_str() ) {
}

String::String(char ch) {
    _str = (char*) malloc( 2 * sizeof(char) );
    _str[0] = ch;
    _str[1] = '\0';
}

String::String(int nb) {
    _str = (char*) malloc( (SIZE_INT16_STR + 1) * sizeof(char) );
    sprintf(_str, "%d", nb);
}

String::String(long nb) {
    _str = (char*) malloc( (SIZE_LONG32_STR + 1) * sizeof(char) );
    sprintf(_str, "%ld", nb);
}

String::~String() {
    free(_str);
}

char String::back() const {
    if (_str == nullptr || length() == 0) {
        return '\0';
    } else {
        return _str[length() - 1];
    }
}

void String::pop_back() {
    if (_str == nullptr || length() == 0) {
        return;
    }
    _str[length() - 1] = '\0'; // TODO maybe free memory ?
}

String& String::operator+=(const char* str) {
    size_t newLen = length() + strlen(str);
    char* newStr = (char*) malloc ((newLen + 1) * sizeof(char));
    if (_str != nullptr) {
        strcpy(newStr, _str);
    }
    strcpy(newStr + length(), str);
    free(_str);
    _str = newStr;
    return *this;
}

String& String::operator+= (char ch) {
    char* newStr = (char*) malloc ((length() + 2) * sizeof(char));
    if (_str != nullptr) {
        strcpy(newStr, _str);
    }
    newStr[length()] = ch;
    newStr[length() + 1] = '\0';
    free(_str);
    _str = newStr;
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

String String::operator+ (char ch) const {
    String newStr(*this);
    return (newStr += ch);
}

void move(String& dest, String& src) {
    free(dest._str);
    dest._str = src._str;
    src._str = nullptr;
}
