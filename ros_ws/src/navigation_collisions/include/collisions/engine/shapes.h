#ifndef COLLISIONS_ENGINE_SHAPES_H
#define COLLISIONS_ENGINE_SHAPES_H

#include "collisions/point.h"
#include "collisions/position.h"

#include <string>

class MapObstacle {
public:
    
    MapObstacle(Position pos, double velocity = 0.0):
        _pos(pos),
        _velocity(velocity)
    {}
    
    const virtual std::string getTypeName() const {
        using namespace std::string_literals;
        return {"map"s};
    };
    
    // Getters & setters
    Position getPos() const { return _pos; }
    void setPos(const Position& pos) { _pos = pos; }
    
    double getVelocity() const { return _velocity; }
    void setVelocity(double velocity) { _velocity = velocity; }
    
protected:
    Position _pos;
    double _velocity;
};

class SegmentObstacle: public MapObstacle {
public:
    SegmentObstacle(const Point& firstPoint, const Point& lastPoint, double velocity = 0.0);
    
    const std::string const std::string getTypeName() const override {
        using namespace std::string_literals;
        return { "segment"s };
    }
    
private:
    Point _first, _last;
    double _length;
    Position _centerPos;
}


#endif // COLLISIONS_ENGINE_SHAPES_H
