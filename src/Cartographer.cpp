
#include "include/Cartographer.h"


Cartographer::Cartographer(int cellsize, int xmin, int ymin, int xmax, int ymax) :
    occupancyGrid(cellsize, xmin, ymin, xmax, ymax), exploredGrid(cellsize, xmin, ymin, xmax,ymax, 0) {
    this->cellSize = cellsize;
}


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
    return cellSize;
}

double Cartographer::GetCellWidth() const {
    return cellSize;
}

void Cartographer::SetPreception(const std::shared_ptr<Perception> &perception) {
    this->perception = perception;
}


glm::ivec2 Cartographer::WorldLocationToCell(const glm::dvec3 &WorldLocation) const {
    return WorldLocationToCell(WorldLocation.x, WorldLocation.y);
}

glm::ivec2 Cartographer::WorldLocationToCell(double x, double y) const {
    int row = std::lround((y - GetYMin())  / (float)cellSize);
    int column = std::lround((x - GetXMin()) / (float) cellSize);
    return {row, column};
}
glm::dvec3 Cartographer::CellToWorldLocation(const glm::ivec2& cell) const {
    return CellToWorldLocation(cell.x, cell.y);
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

bool Cartographer::IsBoxInside(glm::dvec3 LineA, glm::dvec3 LineB,
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


    if (IsOutside(CellLocalLocation, NormalA)) {
        return false;
    }

    if(IsOutside(CellLocalLocation, NormalB)) {
        return false;
    }

//    if(IsOutside(CellLocalLocation, -RobotForward)) {
//        return false;
//    }
    return true;

}

bool Cartographer::IsOutside(glm::dvec3 Point, glm::dvec3 Normal) const {
    return glm::dot(Normal, Point) > 0;
}

bool Cartographer::IsCellWithinBoundry(glm::ivec2 cell, glm::dvec3 LaserVector) const {
    std::vector<glm::dvec3> corners;
    glm::dvec3 cwl = CellToWorldLocation(cell);
    glm::dvec3 cll = cwl - perception->GetLaserLocation();

    glm::dvec3 robotForward = perception->GetRobotForwardVector();

    glm::dvec3 AA = glm::normalize(glm::rotateZ(LaserVector, -Beta));
    glm::dvec3 BB = glm::normalize(glm::rotateZ(LaserVector, Beta));

    return IsBoxInside(AA, BB, cll);

}

glm::dvec3 Cartographer::WorldLaserDirection(double GlobalLaserHeading) const {
    glm::dvec3 xvec(1, 0, 0.0);
    glm::dvec3 HeadingVector = glm::rotateZ(xvec, GlobalLaserHeading);
    return HeadingVector;
}

glm::dvec3 Cartographer::ComputeLocalHitLocation(double Distance, double GlobalLaserHeading) const {
    glm::dvec3 HeadingVector = WorldLaserDirection(GlobalLaserHeading);
    glm::dvec3 LocalHitPosition = HeadingVector * Distance;
    return LocalHitPosition;
}

glm::dvec3
Cartographer::ComputeWorldHitLocation(double Distance, const glm::dvec3 &LaserPosition, double GlobalLaserHeading) const {
    glm::dvec3 LocalHitLocation = ComputeLocalHitLocation(Distance, GlobalLaserHeading);
    glm::dvec3 GlobalHitLocation = LaserPosition + LocalHitLocation;
    return GlobalHitLocation;
}

double Cartographer::DistanceToCell(glm::ivec2 cell, glm::dvec3 position) const {
    return glm::length(CellToWorldLocation(cell) - position);
}

void Cartographer::GetCellsInRange(double range, std::vector<glm::ivec2> &outCells) {
    outCells.clear();
    for (int c = 0; c < occupancyGrid.NumColumns(); ++c) {
        for (int r = 0; r < occupancyGrid.NumRows(); ++r) {
            double distance = glm::length(CellToWorldLocation({r, c}) - perception->GetLaserLocation());
            if (distance < range) {
                outCells.emplace_back(r, c);
            }
        }
    }
}

int Cartographer::MapWidth() const {return occupancyGrid.NumColumns();}

int Cartographer::MapHeight() const {return occupancyGrid.NumRows();}

std::vector<glm::ivec2> Cartographer::GetAdjacent(glm::ivec2 cell) const {
    std::vector<glm::ivec2> adjacents, valids;
    adjacents.emplace_back(cell.x-1, cell.y-1);
    adjacents.emplace_back(cell.x-1, cell.y);
    adjacents.emplace_back(cell.x-1, cell.y+1);

    adjacents.emplace_back(cell.x, cell.y-1);
    adjacents.emplace_back(cell.x, cell.y+1);

    adjacents.emplace_back(cell.x+1, cell.y-1);
    adjacents.emplace_back(cell.x+1, cell.y);
    adjacents.emplace_back(cell.x+1, cell.y+1);
    for(glm::ivec2 c : adjacents) {
        if(occupancyGrid.IsValidCell(c)) {
            valids.push_back(c);
        }
    }
    return valids;
}

std::vector<glm::ivec2> Cartographer::GetNeighbors(glm::ivec2 cell) const {
    std::vector<glm::ivec2> adjacents, valids;
    adjacents.emplace_back(cell.x, cell.y-1);
    adjacents.emplace_back(cell.x, cell.y+1);
    adjacents.emplace_back(cell.x-1, cell.y);
    adjacents.emplace_back(cell.x+1, cell.y);
    for(glm::ivec2 c : adjacents) {
        if(occupancyGrid.IsValidCell(c)) {
            valids.push_back(c);
        }
    }
    return valids;
}


double Cartographer::GetProbabilityEmpty(glm::ivec2 cell) const {
    return 1.0 - occupancyGrid.GetCellValue(cell);
}

bool Cartographer::IsUnknown(glm::ivec2 cell) const {
    return exploredGrid.GetCellValue(cell) < 0.5;
}

void Cartographer::AddKnownCell(const glm::ivec2 &cell) {
    exploredGrid.UpdateCell(cell, 1.0);
}


