
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

double Cartographer::GetCellHeight() const {
    return 1;
}

double Cartographer::GetCellWidth() const {
    return 1;
}

void Cartographer::SetPreception(const std::shared_ptr<Perception> &perception) {
    this->perception = perception;
}

glm::ivec2 Cartographer::WorldLocationToCell(double x, double y) const {
    int row = std::lround(y - GetYMin());
    int column = std::lround(x - GetXMin());
    return {row, column};
}

glm::dvec3 Cartographer::CellToWorldLocation(glm::ivec2 cell) const {
    return CellToWorldLocation(cell.x, cell.y);
}

//glm::dvec3 Cartographer::CellToLocalLocation(glm::ivec2 cell) const {
//    if(!perception.get())
//        return glm::dvec3();
//    glm::dvec3 pos = perception->GetPosition();
//    return CellToWorldLocation(cell) - pos;
//}

glm::ivec2 Cartographer::WorldLocationToCell(const glm::dvec3 &WorldLocation) const {
    return WorldLocationToCell(WorldLocation.x, WorldLocation.y);
}

glm::dvec3 Cartographer::CellToWorldLocation(int row, int col) const {
    double x = (col * GetCellWidth()) + (GetCellWidth() / 2) + GetXMin();
    double y = (row * GetCellHeight()) + (GetCellHeight() / 2) + GetYMin();
    return {x, y, 0};
}

