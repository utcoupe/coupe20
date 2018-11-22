#include "shapes.h"

bool Circle::contains_point(float x, float y) const {
    float diff_x = x - x_;
    float diff_y = y - y;
    float dist_sqr = diff_x * diff_x + diff_y * diff_y;
    return dist_sqr < radius_ * radius_;
}

bool Rectangle::contains_point(float x, float y) const {
    return x >= x_ - width_ / 2
           and x <= x_ + width_ / 2
           and y >= y_ - height_ / 2
           and y <= y_ + height_ / 2;
}