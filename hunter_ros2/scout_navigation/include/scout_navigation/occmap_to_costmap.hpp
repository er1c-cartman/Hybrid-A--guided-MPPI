#ifndef SCOUT_NAVIGATION_COSTMAP_HPP_
#define SCOUT_NAVIGATION_COSTMAP_HPP_

#include <vector>
#include <cmath>  // For std::sqrt
#include <stdexcept>

class Costmap
{
public:
    Costmap() : size_x_(0), size_y_(0), resolution_(0.0), origin_x_(0.0), origin_y_(0.0) {}

    void resizeMap(unsigned int size_x, unsigned int size_y, double resolution, double origin_x, double origin_y)
    {
        size_x_ = size_x;
        size_y_ = size_y;
        resolution_ = resolution;
        origin_x_ = origin_x;
        origin_y_ = origin_y;
        costmap_.resize(size_x_ * size_y_, FREE_SPACE);
    }

    void setCost(unsigned int x, unsigned int y, unsigned char cost)
    {
        if (x >= size_x_ || y >= size_y_)
        {
            throw std::out_of_range("Index out of bounds");
        }
        costmap_[y * size_x_ + x] = cost;
    }

    unsigned char getCost(unsigned int x, unsigned int y) const
    {
        if (x >= size_x_ || y >= size_y_)
        {
            throw std::out_of_range("Index out of bounds");
        }
        return costmap_[y * size_x_ + x];
    }

    // Function to clear the center area (always obstacle-free)
    void clearCenterCosts(double radius)
    {
        unsigned int center_x = size_x_ / 2;
        unsigned int center_y = size_y_ / 2;

        for (unsigned int y = 0; y < size_y_; ++y)
        {
            for (unsigned int x = 0; x < size_x_; ++x)
            {
                double dx = static_cast<double>(x) - center_x;
                double dy = static_cast<double>(y) - center_y;
                double distance = std::sqrt(dx * dx + dy * dy);

                if (distance <= radius)
                {
                    setCost(x, y, FREE_SPACE);  // Clear the costs in the circular area
                }
            }
        }
    }

private:
    unsigned int size_x_;
    unsigned int size_y_;
    double resolution_;
    double origin_x_;
    double origin_y_;
    std::vector<unsigned char> costmap_;

    const unsigned char FREE_SPACE = 0;
    const unsigned char LETHAL_OBSTACLE = 254;
    const unsigned char NO_INFORMATION = 255;
};

#endif  // SCOUT_NAVIGATION_COSTMAP_HPP_
