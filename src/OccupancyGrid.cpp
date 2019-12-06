

#include "include/OccupancyGrid.h"

OccupancyGrid::OccupancyGrid(int xmin, int ymin, int xmax, int ymax)
        : Xmin(xmin), Xmax(xmax), Ymin(ymin), Ymax(ymax)
{
    int width = xmax - xmin;
    int height = ymax - ymin;
    Grid = std::vector<std::vector<double>>(width, std::vector<double>(height, 0.5));
}
