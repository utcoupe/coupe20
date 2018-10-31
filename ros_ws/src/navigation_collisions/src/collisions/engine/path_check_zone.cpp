#include "collisions/engine/path_check_zone.h"

#include "collisions/shapes/rectangle.h"
#include "collisions/shapes/circle.h"
#include "collisions/engine/engine.h"
#include "collisions/engine/constants.h"

using namespace CollisionsShapes;

std::vector<PathCheckZone::ShapePtr> PathCheckZone::getShapes(Position robotPos) {
    if (waypoints_.empty())
        return {};
    std::vector<ShapePtr> shapes;
    auto path = getFullWaypoints(robotPos);
    for (unsigned idPos = 1; idPos < path.size(); idPos++) {
        const auto& prevPos = (idPos == 0 ? robotPos : path[idPos-1]);
        const auto& curPos = path[idPos];
        
        if (prevPos == curPos)
            continue;
        
        double dist = prevPos.norm2Dist(curPos);
        double angle = std::atan((curPos.getY() - prevPos.getY()) / (curPos.getX() - prevPos.getX()));
        Position pos(
            (curPos.getX() + prevPos.getX()) / 2,
            (curPos.getY() + prevPos.getY())/2,
            angle
        );
        
        shapes.push_back(std::make_shared<Rectangle>(pos, dist, width_));
        if (idPos + 1 == path.size()) {
            shapes.push_back(std::make_shared<Rectangle>(
                Position(curPos.getX(), curPos.getY(), angle),
                height_,
                width_
            ));
        }
        else {
            double ray = std::sqrt(width_ * width_ + height_ * height_) / 2.0;
            shapes.push_back(std::make_shared<Circle>(
                curPos,
                ray
            ));
        }
    }
    return shapes;
}

std::vector<Collision> PathCheckZone::checkCollisions(Position robotPos, std::vector<ObstaclePtr> obstacles)
{
    std::vector<Collision> collisions;
    auto shapes = getShapes(robotPos);
    for (auto obst: CollisionResolver::findCollisions(shapes, obstacles)) {
        double approxDist = robotPos.norm2Dist(obst->getPos());
        if (approxDist < CollisionThresholds::DANGER_RADIUS) 
            collisions.emplace_back(CollisionLevel::LEVEL_DANGER, obst, approxDist);
        else
            collisions.emplace_back(CollisionLevel::LEVEL_POTENTIAL, obst, approxDist);
    }
    return collisions;
}

std::vector<Position> PathCheckZone::getFullWaypoints(Position robotPos) const
{
    std::vector<Position> path = { robotPos };
    path.insert(path.begin(), waypoints_.begin(), waypoints_.end());
    return path;
}
