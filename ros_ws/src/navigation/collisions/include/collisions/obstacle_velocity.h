#ifndef COLLISIONS_OBSTACLE_VELOCITY_H
#define COLLISIONS_OBSTACLE_VELOCITY_H

#include "collisions/shapes/abstract_shape.h"

#include <memory>
#include <vector>

class ObstacleVelocity {
public:
    using ShapePtr = std::unique_ptr<CollisionsShapes::AbstractShape>;
    
    ObstacleVelocity(double width, double height, double velLinear = 0.0, double velAngular = 0.0, Position objectPos = {}) noexcept;
    
    const std::vector<ShapePtr>& getShapes(double maxDist = -1.0) const;
    
    void setObjectPos(const Position& pos) noexcept {
        m_objectPos = pos;
        m_needUpdate = true;
    }
    
    void setVelocity(double velLinear, double velAngular) {
        m_velLinear = velLinear;
        m_velAngular = velAngular;
        m_needUpdate = true;
    }
    
private:
    mutable bool m_needUpdate = true;
    double m_width, m_height, m_velLinear, m_velAngular;
    Position m_objectPos;
    mutable std::vector<ShapePtr> m_velShapes;
    mutable double m_lastMaxDist = -1.0;
    
    void m_generateVelShapes(double maxDist) const;
};

#endif // COLLISIONS_OBSTACLE_VELOCITY_H
