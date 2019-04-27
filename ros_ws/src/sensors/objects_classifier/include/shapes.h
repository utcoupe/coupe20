#ifndef objects_classifier_SHAPES_H
#define objects_classifier_SHAPES_H

#include <geometry_tools/point.h>

class Shape {
public:
    virtual bool contains_point(Point point) const = 0;

    Shape(Point pos = {}):
        _pos(pos)
    {}

    Shape(const Shape &other) {}
    
protected:
    Point _pos;
};

class Circle : public Shape {
protected:
    double radius_;
public:
    Circle(Point pos, double radius) :
        Shape(pos),
        radius_(radius) {}

    bool contains_point(Point point) const override;
};

class Rectangle : public Shape {
protected:
    double width_, height_;
public:
    Rectangle(Point pos, double width, double height) :
            Shape(pos),
            width_(width),
            height_(height)
    {}

    bool contains_point(Point point) const override;
};

// TODO: polygon

#endif //objects_classifier_SHAPES_H
