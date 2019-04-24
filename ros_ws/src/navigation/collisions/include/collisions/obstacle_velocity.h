#ifndef COLLISIONS_OBSTACLE_VELOCITY_H
#define COLLISIONS_OBSTACLE_VELOCITY_H

#include "collisions/shapes/abstract_shape.h"

#include <memory>
#include <vector>

class ObstacleVelocity {
public:
    using ShapePtr = std::shared_ptr<CollisionsShapes::AbstractShape>;
    
    ObstacleVelocity(double width, double height, double velLinear = 0.0, double velAngular = 0.0, Position objectPos = {}) noexcept;
    
    std::vector<ShapePtr> getShapes(double maxDist = -1.0);
    
    void setObjectPos(Position pos) noexcept {
        m_objectPos = pos;
        m_needUpdate = true;
    }
    
    void setVelocity(double velLinear, double velAngular) {
        m_velLinear = velLinear;
        m_velAngular = velAngular;
        m_needUpdate = true;
    }
    
private:
    bool m_needUpdate = true;
    double m_width, m_height, m_velLinear, m_velAngular;
    Position m_objectPos;
    std::vector<ShapePtr> m_velShapes;
    double m_lastMaxDist = -1.0;
    
    void m_generateVelShapes(double maxDist);
};

#endif // COLLISIONS_OBSTACLE_VELOCITY_H
