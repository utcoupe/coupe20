#include "pathfinder/dynamic_barriers_manager.h"

#include "pathfinder/pos_convertor.h"

#include <geometry_tools/point.h>

#include <utility>

using namespace std;

bool DynamicBarriersManager::hasBarriers(const Point& pos) const {
    auto posConverted = _convertor.fromMapToRosPos(pos);
    
    for (const auto& subscriber : subscribers)
    {
        if (subscriber->needConversionBefore() && subscriber->hasBarrier(posConverted))
            return true;
        else if (!subscriber->needConversionBefore() && subscriber->hasBarrier(pos))
            return true;
    }
    return false;
}


void DynamicBarriersManager::addBarrierSubscriber(BarriersSubscriber && subscriber)
{
    subscribers.push_back(std::move(subscriber));
}

void DynamicBarriersManager::updateSafetyMargin(const double& newMargin)
{
    for (auto& subscriber : subscribers)
        subscriber->setSafetyMargin(newMargin);
}

void DynamicBarriersManager::fetchOccupancyDatas(
    const uint& widthGrid,
    const uint& heightGrid,
    const std::vector<std::string>& ignoredTags
) const {
    for (const auto& subscriber : subscribers)
        subscriber->fetchOccupancyData(widthGrid, heightGrid, ignoredTags);
}

