
#include "include/Cartographer.h"

int Cartographer::GetXMin() const {
    return occupancyGrid.Xmin;
}

int Cartographer::GetYMin() const {
    return occupancyGrid.Ymin;
}

const std::vector<std::vector<double>> &Cartographer::GetProbablityGrid() const {
    return occupancyGrid.GetGrid();
}
