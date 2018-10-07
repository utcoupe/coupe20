#include "collisions/position.h"

using namespace std;

Point Position::toPoint() const {
    return {_x, _y};
}

bool Position::operator== (const Position& other) const {
    return this->toPoint() == other.toPoint() && _a == other.getAngle();
}

bool Position::operator!= (const Position& other) const {
    return !this->operator==(other);
}
