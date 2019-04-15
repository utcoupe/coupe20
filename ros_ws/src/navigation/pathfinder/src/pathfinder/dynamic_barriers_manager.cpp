#include "pathfinder/dynamic_barriers_manager.h"

#include <utility>

using namespace std;

bool DynamicBarriersManager::hasBarriers(const Point& pos)
{
    auto posConverted = _convertor->fromMapToRosPos(pos);
    
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

void DynamicBarriersManager::setConvertor(const shared_ptr<PosConvertor>& convertor)
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

