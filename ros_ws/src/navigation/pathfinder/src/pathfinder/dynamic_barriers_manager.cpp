#include "pathfinder/dynamic_barriers_manager.h"

#include <utility>

using namespace std;

bool DynamicBarriersManager::hasBarriers(const geometry_msgs::Pose2D& pos)
{
    for (const auto& subscriber : subscribers)
        if (subscriber->needConversionBefore() && subscriber->hasBarrier(pos))
            return true;
    return false;
}

bool DynamicBarriersManager::hasBarriers(const Point& pos)
{
    auto pose2dConvert = pointToPose2D(pos);
    geometry_msgs::Pose2D pose2d;
    pose2d.x = pos.getX();
    pose2d.y = pos.getY();
    
    for (const auto& subscriber : subscribers)
    {
        if (subscriber->needConversionBefore() && subscriber->hasBarrier(pose2dConvert))
            return true;
        else if (!subscriber->needConversionBefore() && subscriber->hasBarrier(pose2d))
            return true;
    }
    return false;
}


void DynamicBarriersManager::addBarrierSubscriber(BarriersSubscriber && subscriber)
{
    subscribers.push_back(std::move(subscriber));
}

void DynamicBarriersManager::setConvertor(std::shared_ptr<PosConvertor> convertor)
{
    _convertor = convertor;
}

void DynamicBarriersManager::updateSafetyMargin(const double& newMargin)
{
    for (auto& subscriber : subscribers)
        subscriber->setSafetyMargin(newMargin);
}

void DynamicBarriersManager::fetchOccupancyDatas(const uint& widthGrid, const uint& heightGrid) const
{
    for (const auto& subscriber : subscribers)
        subscriber->fetchOccupancyData(widthGrid, heightGrid);
}

geometry_msgs::Pose2D DynamicBarriersManager::pointToPose2D(const Point& pos) const
{
    auto convertedPos = _convertor->fromMapToRosPos(pair<double, double>(pos.getX(), pos.getY()));
    geometry_msgs::Pose2D newPos;
    newPos.x = convertedPos.first;
    newPos.y = convertedPos.second;
    return newPos;
}

