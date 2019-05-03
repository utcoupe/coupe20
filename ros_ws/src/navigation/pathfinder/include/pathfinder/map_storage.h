#ifndef MAP_STORAGE_H
#define MAP_STORAGE_H

#include <ros/console.h>

#include <SFML/Graphics/Image.hpp>

#include <memory>
#include <vector>

namespace pathfinder {
    class OccupancyGrid;
} // namespace pathfinder

class Point;

class DynamicBarriersManager;

/**
 * Class used to load and save the pathfinder's datas in an image format.
 */
class MapStorage
{
public:
    using Vect2DBool = std::vector<std::vector<bool>>;
    
    MapStorage() = default;
    
    /**
     * Loads from an image the allowed positions. Supported format are bmp, png, tga, jpg, gif, psd, hdr and pic.
     * @param filename The path to the file to load.
     * @return A 2D boolean grid with the same size as the loaded image, true is meaning no obstacle in the case.
     */
    Vect2DBool loadAllowedPositionsFromFile(const std::string& fileName);
    
    /**
     * Saves the allowed position and the pathes to an image. Supported format are bmp, png, tga and jpg.
     * @param filename The place and the filename where to create or replace the image.
     * @param allowedPos A 2D grid with the same size than the image to create, true is meaning no obstacle in the case.
     * @param dynBarriersMng The obstacle subscribers manager.
     * @param path The raw path at the end of the pathfinder algorithm
     * @param smoothPath The smoothed path that will be send as response.
     */
    void saveMapToFile(const std::string& fileName, const pathfinder::OccupancyGrid& allowedPos, const DynamicBarriersManager& dynBarriersMng, const std::vector< Point >& path, const std::vector< Point >& smoothPath) const;

private:
    const sf::Color ALLOWED_POS_COLOR       = sf::Color::White;
    const sf::Color NOT_ALLOWED_POS_COLOR   = sf::Color::Black;
    const sf::Color DYN_BARRIER_COLOR       = sf::Color::Red;
    const sf::Color PATH_COLOR              = sf::Color::Blue;
    const sf::Color SMOOTH_PATH_COLOR       = sf::Color::Green;
    
    void drawPath(sf::Image& image, const Point& pA, const Point& pB, const sf::Color& color) const;
};


#endif // MAP_STORAGE_H
