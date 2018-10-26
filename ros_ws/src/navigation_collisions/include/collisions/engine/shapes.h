#ifndef COLLISIONS_ENGINE_SHAPES_H
#define COLLISIONS_ENGINE_SHAPES_H

#include "collisions/point.h"
#include "collisions/position.h"
#include "collisions/engine/check_zone.h"

#include <chrono>
#include <memory>
#include <vector>

enum class ObstacleTypes {UNDEFINED, SEGMENT, RECTANGLE, CIRCLE};

class MapObstacle;
using PtrObstacle = std::shared_ptr<MapObstacle>;
using PtrVelocity = std::shared_ptr<CheckZone>;

class MapObstacle {
public:
    MapObstacle(Position pos = {0.0, 0.0, 0.0}, PtrVelocity velocity = nullptr)
    {
        initializeMapObstacle(pos, velocity);
    }
    
    const virtual ObstacleTypes getTypeName() const = 0;
    
    // Getters & setters
    Position getPos() const { return _pos; }
    void setPos(const Position& pos) { _pos = pos; }
    
    PtrVelocity getVelocity() const { return _velocity; }
    void setVelocity(PtrVelocity velocity) { _velocity = velocity; }
    
protected:
    Position _pos;
    PtrVelocity _velocity;
    std::chrono::system_clock::time_point _spawnTime = std::chrono::system_clock::now();
    
    void initializeMapObstacle(Position pos, PtrVelocity velocity);
};

class SegmentObstacle: public MapObstacle {
public:
    SegmentObstacle(const Point& firstPoint, const Point& lastPoint, PtrVelocity velocity = nullptr);
    
    const ObstacleTypes getTypeName() const override {
        return ObstacleTypes::SEGMENT;
    }
    
    std::pair<Point,Point> segment() const { return {_first, _last}; }
    
    Point getFirst() const { return _first; }
    Point getLast() const { return _last; }
    double getLength() const { return _length; }
    
private:
    Point _first, _last;
    double _length;
    Position _centerPos;
};

class CircleObstacle: public MapObstacle {
public:
    CircleObstacle(Position pos, double radius, PtrVelocity velocity = nullptr):
        MapObstacle(pos, velocity),
        _radius(radius)
    {}
    
    const ObstacleTypes getTypeName() const override {
        return ObstacleTypes::CIRCLE;
    }
    
    double getRadius() const { return _radius; }
    
private:
    double _radius;
};

class RectObstacle: public MapObstacle {
public:
    RectObstacle(Position pos, double width, double height, PtrVelocity velocity = nullptr):
        MapObstacle(pos, velocity),
        _width(width),
        _height(height)
    {}
    
    std::vector<SegmentObstacle> toSegments() const;
    
    const ObstacleTypes getTypeName() const override {
        return ObstacleTypes::RECTANGLE;
    }
    
    double getWidth() const { return _width; }
    double getHeight() const { return _height; }
        
private:
    double _width, _height;
    
    ROS_DEPRECATED std::vector<Point> getCorners() const; // Calculs perso de @MadeInPierre, à vérifier
};

#endif // COLLISIONS_ENGINE_SHAPES_H
