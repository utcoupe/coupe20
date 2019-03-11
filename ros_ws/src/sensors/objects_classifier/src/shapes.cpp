#include "shapes.h"

bool Circle::contains_point(Point point) const {
    return point.norm2Dist(_pos) < radius_;
}

bool Rectangle::contains_point(Point point) const {
    return point.getX() >= _pos.getX() - width_ / 2
           && point.getX() <= _pos.getX() + width_ / 2
           && point.getY() >= _pos.getY() - height_ / 2
           && point.getY() <= _pos.getY() + height_ / 2;
}
