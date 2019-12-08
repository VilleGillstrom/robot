
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

glm::dvec3 Cartographer::GetVertexP(const glm::vec3 &normal, glm::dvec3 cll) const {
    glm::vec3 point;
    glm::dvec3 TR = TopRightCellCorner(cll);
    glm::dvec3 BL = BotLeftCellCorner(cll);

    point.x = normal.x >= 0 ? TR.x : BL.x;
    point.y = normal.y >= 0 ? TR.y : BL.y;
    return  point;
}

glm::dvec3 Cartographer::GetVertexN(const glm::dvec3 &normal, glm::dvec3 cll) const {
    glm::vec3 point;
    glm::dvec3 TR = TopRightCellCorner(cll);
    glm::dvec3 BL = BotLeftCellCorner(cll);

    point.x = normal.x >= 0 ? BL.x : TR.x;
    point.y = normal.y >= 0 ? BL.y : TR.y;
    return  point;
}

bool Cartographer::IsBoxInside(glm::dvec3 RobotForward, glm::dvec3 LineA, glm::dvec3 LineB,
                               glm::dvec3 CellLocalLocation) const {

    glm::vec3 NormalA = glm::cross( glm::normalize(LineA),glm::dvec3(0, 0, 1));
    glm::vec3 NormalB = glm::cross( glm::dvec3(0, 0, 1), glm::normalize(LineB));

    //glm::dvec3 NA = GetVertexN(NormalA, CellLocalLocation);
   // glm::dvec3 NB = GetVertexN(NormalB, CellLocalLocation);
    //glm::dvec3 NRobot = GetVertexN(-RobotForward, CellLocalLocation);

//    if (IsOutside(NA, NormalA)) {
//        return false;
//    }
//
//    if(IsOutside(NB, NormalB)) {
//        return false;
//    }
//
//    if(IsOutside(NRobot, -RobotForward)) {
//        return false;
//    }

    if (IsOutside(CellLocalLocation, NormalA)) {
        return false;
    }

    if(IsOutside(CellLocalLocation, NormalB)) {
        return false;
    }

    if(IsOutside(CellLocalLocation, -RobotForward)) {
        return false;
    }
    return true;

}

bool Cartographer::IsOutside(glm::dvec3 Point, glm::dvec3 Normal) const {
    return glm::dot(Normal, Point) > 0;
}

bool Cartographer::IsCellWithinBoundry(glm::ivec2 cell, glm::dvec3 LaserVector, double Beta) const {
    std::vector<glm::dvec3> corners;
    glm::dvec3 cwl = CellToWorldLocation(cell);
    glm::dvec3 cll = cwl - perception->GetLaserPosition();

    glm::dvec3 robotForward = perception->GetRobotForwardVector();

    glm::dvec3 AA = glm::normalize(glm::rotateZ(LaserVector, -Beta));
    glm::dvec3 BB = glm::normalize(glm::rotateZ(LaserVector, Beta));

    return IsBoxInside(robotForward, AA, BB, cll);

}

