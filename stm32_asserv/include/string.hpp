#ifndef STRING_H
#define STRING_H

#include <cstdint>

class String {
public:
    String();
    String(const char* str);
    String(const String& str):
        String( str.c_str() ) {
    }
    ~String();
    
    char back() const;
    const char* c_str() const { return _str; };
    std::size_t length() const;
    void pop_back();
    
    String& operator+= (const char* str);
    String& operator+= (const String& str) { return this->operator+=(str.c_str()); }
    bool operator== (const String& other) const;
    bool operator!= (const String& other) const { return !( this->operator==(other) ); }
    String operator+ (const char* str) const;
    String operator+ (const String& str) const;
    
    String& operator= (const char* str);
    String& operator= (const String& str) { return this->operator=(str.c_str()); }
    
private:
    char* _str;
};

#endif // STRING_H
