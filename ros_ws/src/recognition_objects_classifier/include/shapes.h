
#ifndef RECOGNITION_OBJECTS_CLASSIFIER_SHAPES_H
#define RECOGNITION_OBJECTS_CLASSIFIER_SHAPES_H

class Shape {
public:
    virtual bool contains_point(float x, float y) const = 0;

    Shape() {}

    Shape(const Shape &other) {}
};

class Circle : public Shape {
protected:
    float x_, y_, radius_;
public:
    Circle(float x, float y, float radius) :
            x_(x), y_(y), radius_(radius) {}

    bool contains_point(float x, float y) const;
};

class Rectangle : public Shape {
protected:
    float x_, y_, width_, height_;
public:
    Rectangle(float x, float y, float width, float height) :
            x_(x), y_(y), width_(width), height_(height) {}

    bool contains_point(float x, float y) const;
};

// TODO: polygon

#endif //RECOGNITION_OBJECTS_CLASSIFIER_SHAPES_H
