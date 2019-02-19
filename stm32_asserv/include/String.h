#ifndef STRING_H
#define STRING_H

#include <stdint.h>
#include <string.h>

class String {
public:
    String();
    String(const char* str);
    String(const String& str);
    String(char ch);
    String(int nb);
    String(long nb);
    ~String();
    
    char back() const;
    const char* c_str() const { return _str; };
    size_t length() const { return strlen(_str); }
    void pop_back();
    
    String& operator+= (const char* str);
    String& operator+= (const String& str) { return this->operator+=(str.c_str()); }
    String& operator+= (char ch);
    bool operator== (const String& other) const;
    bool operator!= (const String& other) const { return !( this->operator==(other) ); }
    String operator+ (const char* str) const;
    String operator+ (const String& str) const;
    String operator+ (char ch) const;
    
    friend void move(String& dest, String& src);
    
private:
    char* _str;
};

#endif // STRING_H
