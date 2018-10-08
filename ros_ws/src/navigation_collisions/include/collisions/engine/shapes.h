#ifndef COLLISIONS_ENGINE_SHAPES_H
#define COLLISIONS_ENGINE_SHAPES_H

#include "collisions/point.h"
#include "collisions/position.h"

#include <chrono>
#include <string>
#include <vector>

class MapObstacle {
public:
    MapObstacle(Position pos = {0.0, 0.0, 0.0}, double velocity = 0.0)
    {
        initializeMapObstacle(pos, velocity);
    }
    
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
    std::chrono::system_clock::time_point _spawnTime = std::chrono::system_clock::now();
    
    void initializeMapObstacle(Position pos, double velocity);
};

class SegmentObstacle: public MapObstacle {
public:
    SegmentObstacle(const Point& firstPoint, const Point& lastPoint, double velocity = 0.0);
    
    const std::string getTypeName() const override {
        using namespace std::string_literals;
        return { "segment"s };
    }
    
    std::pair<Point,Point> segment() const { return {_first, _last}; }
    
private:
    Point _first, _last;
    double _length;
    Position _centerPos;
};

class CircleObstacle: public MapObstacle {
public:
    CircleObstacle(Position pos, double width, double height, double velocity = 0.0):
        MapObstacle(pos, velocity),
        _width(width),
        _height(height)
    {}
    
    std::vector<SegmentObstacle> toSegments() const;
    
    const std::string getTypeName() const override {
        using namespace std::string_literals;
        return {"circle"s};
    }
    
private:
    double _width, _height;
    
    ROS_DEPRECATED std::vector<Point> getCorners() const; // Calculs perso de @MadeInPierre, à vérifier
};

class RectObstacle: public MapObstacle {
public:
    RectObstacle(Position pos, double width, double height, double velocity = 0.0):
        MapObstacle(pos, velocity),
        _width(width),
        _height(height)
    {}
    
    std::vector<SegmentObstacle> toSegments() const;
    
    const std::string getTypeName() const override {
        using namespace std::string_literals;
        return {"rect"s};
    }
    
    double getWidth() const { return _width; }
    double getHeight() const { return _height; }
        
private:
    double _width, _height;
    
    std::vector<Point> getCorners() const;
};

#endif // COLLISIONS_ENGINE_SHAPES_H
