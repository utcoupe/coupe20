#include "pathfinder/map_storage.h"

#include <cmath>

using namespace std;

MapStorage::Vect2DBool MapStorage::loadAllowedPositionsFromFile(const string& fileName)
{
    ROS_DEBUG_STREAM("MapStorage: loading " << fileName);
    
    sf::Image image;
    image.loadFromFile(fileName);
    
    Vect2DBool allowedPos;
    for (unsigned int line = 0; line < image.getSize().y; line++)
    {
        allowedPos.emplace_back();
        for (unsigned int column = 0; column < image.getSize().x; column++)
            allowedPos[line].push_back(image.getPixel(column, line) == ALLOWED_POS_COLOR);
    }
    ROS_DEBUG_STREAM("MapStorage: Done, map size is " << allowedPos.front().size() << "x" << allowedPos.size());
    return allowedPos;
}

void MapStorage::saveMapToFile(const string& fileName, const Vect2DBool& allowedPos, shared_ptr<DynamicBarriersManager> dynBarriersMng, const vector<Point>& path, const vector<Point>& smoothPath)
{
    ROS_DEBUG_STREAM("MapStorage: saving to " << fileName);
    sf::Image image;
    image.create(allowedPos.front().size(), allowedPos.size());
    
    for (unsigned int line = 0; line < allowedPos.size(); line++)
    {
        for (unsigned int column = 0; column < allowedPos.front().size(); column++)
        {
            if (!allowedPos[line][column])
                image.setPixel(column, line, NOT_ALLOWED_POS_COLOR);
            else if (dynBarriersMng->hasBarriers(Point(column, line)))
                image.setPixel(column, line, DYN_BARRIER_COLOR);
            else
                image.setPixel(column, line, ALLOWED_POS_COLOR);
        }
    }
    
    Point lastPos = Point(-1, -1);
    for (const Point& pos : path)
    {
        if (lastPos.getX() != -1)
            drawPath(image, lastPos, pos, PATH_COLOR);
        lastPos = pos;
    }
    lastPos = Point(-1, -1);
    for (const Point& pos : smoothPath)
    {
        if (lastPos.getX() != -1)
            drawPath(image, lastPos, pos, SMOOTH_PATH_COLOR);
        lastPos = pos;
    }
    
    image.saveToFile(fileName);
    ROS_DEBUG("MapStorage: Done");
}

void MapStorage::drawPath(sf::Image& image, const Point& pA, const Point& pB, const sf::Color& color)
{
    Point drawPos = pA;
    int dX, dY, stepX, stepY, error;
    
    dX = abs(pB.getX() - pA.getX());
    dY = abs(pB.getY() - pA.getY());
    
    stepX = (pB.getX() > pA.getX()) ? 1 : -1;
    stepY = (pB.getY() > pA.getY()) ? 1 : -1;
    
    
    image.setPixel(pA.getX(), pA.getY(), color);
    
    if (dX > dY)
    {
        error = dX/2;
        for (int i = 0; i < dX; i++)
        {
            drawPos = drawPos + Point(stepX, 0);
            error += dY;
            if (error > dX)
            {
                error -= dX;
                drawPos = drawPos + Point(0, stepY);
            }
            image.setPixel(drawPos.getX(), drawPos.getY(), color);
        }
    }
    else
    {
        error = dY/2;
        for (int i = 0; i < dY; i++)
        {
            drawPos = drawPos + Point(0, stepY);
            error += dX;
            if (error > dY)
            {
                error -= dY;
                drawPos = drawPos + Point(stepX, 0);
            }
            image.setPixel(drawPos.getX(), drawPos.getY(), color);
        }
    }
    
    image.setPixel(pB.getX(), pB.getY(), color);
}
