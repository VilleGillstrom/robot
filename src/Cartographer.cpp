
#include "include/Cartographer.h"


Cartographer::Cartographer(double cellsize, int xmin, int ymin, int xmax, int ymax) :
        occupancyGrid(cellsize, xmin, ymin, xmax, ymax), exploredGrid(cellsize, xmin, ymin, xmax, ymax, 0) {
    this->cellSize = cellsize;
}


int Cartographer::GetXMin() const {
    return occupancyGrid.Xmin;
}

int Cartographer::GetYMin() const {
    return occupancyGrid.Ymin;
}

int Cartographer::GetXMax() const {
    return occupancyGrid.Xmax;
}

int Cartographer::GetYMax() const {
    return occupancyGrid.Ymax;
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
    int row = std::lround((y - GetYMin()) / (double) cellSize);
    int column = std::lround((x - GetXMin()) / (double) cellSize);

    row = std::max(row, 0);
    row = std::min(row, GetYMax() - GetYMin());

    column = std::max(column, 0);
    column = std::min(column, GetXMax() - GetXMin());

    return {row, column};
}

glm::dvec3 Cartographer::CellToWorldLocation(const glm::ivec2 &cell) const {
    return CellToWorldLocation(cell.x, cell.y);
}

glm::dvec3 Cartographer::CellToWorldLocation(int row, int col) const {
    double x = (col * GetCellWidth()) + (GetCellWidth() / 2.0) + GetXMin();
    double y = (row * GetCellHeight()) + (GetCellHeight() / 2.0) + GetYMin();
    return {x, y, 0};
}


bool Cartographer::IsBoxInside(glm::dvec3 LineA, glm::dvec3 LineB,
                               glm::dvec3 CellLocalLocation) const {

    glm::vec3 NormalA = glm::cross(glm::normalize(LineA), glm::dvec3(0, 0, 1));
    glm::vec3 NormalB = glm::cross(glm::dvec3(0, 0, 1), glm::normalize(LineB));

    if (IsOutside(CellLocalLocation, NormalA)) {
        return false;
    }

    if (IsOutside(CellLocalLocation, NormalB)) {
        return false;
    }


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
Cartographer::ComputeWorldHitLocation(double Distance, const glm::dvec3 &LaserPosition,
                                      double GlobalLaserHeading) const {
    glm::dvec3 LocalHitLocation = ComputeLocalHitLocation(Distance, GlobalLaserHeading);
    glm::dvec3 GlobalHitLocation = LaserPosition + LocalHitLocation;
    return GlobalHitLocation;
}

double Cartographer::DistanceToCell(glm::ivec2 cell, glm::dvec3 position) const {
    return glm::length(CellToWorldLocation(cell) - position);
}

void Cartographer::GetCellsInRobotRange(double range, std::vector<glm::ivec2> &outCells) {
    const glm::dvec3 robotLocation = RobotLocation();
    GetCellsInRange(robotLocation, range, outCells);
}

void Cartographer::GetCellsInRange(const glm::dvec3 &worldlocation, double range, std::vector<glm::ivec2> &outCells) const {
    outCells.clear();

    for (int c = 0; c < occupancyGrid.NumColumns(); ++c) {
        for (int r = 0; r < occupancyGrid.NumRows(); ++r) {
            double distance = glm::length(CellToWorldLocation({r, c}) - worldlocation);
            if (distance < range) {
                outCells.emplace_back(r, c);
            }
        }
    }
}

void Cartographer::GetCellsInRange(const glm::ivec2 & cell, double range, std::vector<glm::ivec2> &outCells) const {
    const glm::dvec3 worldlocation = CellToWorldLocation(cell);
    GetCellsInRange(worldlocation, range, outCells);
}


int Cartographer::MapWidth() const { return occupancyGrid.NumColumns(); }

int Cartographer::MapHeight() const { return occupancyGrid.NumRows(); }

std::vector<glm::ivec2> Cartographer::GetAdjacent(glm::ivec2 cell) const {
    std::vector<glm::ivec2> adjacents, valids;
    adjacents.emplace_back(cell.x - 1, cell.y - 1);
    adjacents.emplace_back(cell.x - 1, cell.y);
    adjacents.emplace_back(cell.x - 1, cell.y + 1);

    adjacents.emplace_back(cell.x, cell.y - 1);
    adjacents.emplace_back(cell.x, cell.y + 1);

    adjacents.emplace_back(cell.x + 1, cell.y - 1);
    adjacents.emplace_back(cell.x + 1, cell.y);
    adjacents.emplace_back(cell.x + 1, cell.y + 1);
    for (glm::ivec2 c : adjacents) {
        if (occupancyGrid.IsValidCell(c)) {
            valids.push_back(c);
        }
    }
    return valids;
}

std::vector<glm::ivec2> Cartographer::GetEmptyAdjacent(glm::ivec2 cell) const {
    std::vector<glm::ivec2> adjacents = GetAdjacent(cell);
    std::vector<glm::ivec2> empty_adjacents;
    for (glm::ivec2 c : adjacents) {
        if (occupancyGrid.GetCellValue(c) < 0.5) {
            empty_adjacents.push_back(c);
        }
    }
    return empty_adjacents;
}

std::vector<glm::ivec2> Cartographer::GetNeighborsLessThan(glm::ivec2 cell, float p) const {
    std::vector<glm::ivec2> adjacents = GetNeighbors(cell);
    std::vector<glm::ivec2> empty_adjacents;
    for (glm::ivec2 c : adjacents) {
        if (occupancyGrid.GetCellValue(c) < p) {
            empty_adjacents.push_back(c);
        }
    }
    return empty_adjacents;
}



std::vector<glm::ivec2> Cartographer::GetNeighbors(glm::ivec2 cell) const {
    std::vector<glm::ivec2> adjacents, valids;
    adjacents.emplace_back(cell.x, cell.y - 1);
    adjacents.emplace_back(cell.x, cell.y + 1);
    adjacents.emplace_back(cell.x - 1, cell.y);
    adjacents.emplace_back(cell.x + 1, cell.y);
    for (glm::ivec2 c : adjacents) {
        if (occupancyGrid.IsValidCell(c)) {
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

double Cartographer::ComputeProbability(double distance, double alpha, double pMax) const {
    double R = 40; // Max distance
    double r = distance;    // distance to hit
    double a = alpha;           //

    double Rratio = (R - r) / R;
    double AngleRatio = (Beta - a) / Beta;
    return ((Rratio + AngleRatio) / 2.0) * pMax;
}

void Cartographer::GetRegions(const glm::dvec3 LaserLocation, const glm::dvec3 WorldHitLocation,
                              const glm::dvec3 &LaserVector, const std::vector<glm::ivec2> &CellsInRange,
                              std::vector<glm::ivec2> &OutRegionI, std::vector<glm::ivec2> &OutRegionII) {

    OutRegionI.clear();
    OutRegionII.clear();

    const std::vector<std::vector<double>> &grid = occupancyGrid.GetGrid();
    for (const glm::ivec2 &cell : CellsInRange) {
        glm::dvec3 cellWorldLocation = CellToWorldLocation(cell);
        glm::dvec3 cellLocalLocation = cellWorldLocation - LaserLocation;

        double RobotDistanceToCell = glm::length(cellLocalLocation);
        double RobotDistanceToHit = glm::length(LaserLocation - WorldHitLocation);

        //Region 3
        if (RobotDistanceToCell > RobotDistanceToHit) { // if cell is beyond the hit skip cell
            continue;
        }

        bool isWithin = IsCellWithinBoundry(cell, LaserVector);
        if (!isWithin) {
            continue; //Cell is outside  beta
        }

        double foo = glm::distance(RobotDistanceToCell, RobotDistanceToHit);

        double CellDistanceFromRobot = glm::length(cellLocalLocation - (WorldHitLocation - LaserLocation));
        double CellDistanceFromHit = glm::length(cellLocalLocation - (WorldHitLocation - LaserLocation));
        if (foo < Region1Range) {
            OutRegionI.push_back(cell);
        } else {
            OutRegionII.push_back(cell);
        }
        AddKnownCell(cell);
    }
}

void Cartographer::HandleEcho(double distanceToHit, const glm::dvec3 &LaserVector,
                              const std::vector<glm::ivec2> &CellsInRange) {
    glm::dvec3 LaserLocation = perception->GetLaserLocation();
    LaserLocation.z = 0; //
    const glm::dvec3 WorldHitLocation = LaserLocation + LaserVector * distanceToHit;
    //std::cerr << alpha << std::endl;

    std::vector<glm::ivec2> RegionI;
    std::vector<glm::ivec2> RegionII;
    GetRegions(LaserLocation, WorldHitLocation, LaserVector, CellsInRange, RegionI, RegionII);

    for (const glm::ivec2 &cell : CellsInRange) {
        //occupancyGrid.UpdateCell(cell, 0.0);
    }

    //Cells in regionI (possible walls)
    for (const auto &cell : RegionI) {
        double alpha = glm::angle(LaserVector, glm::normalize(CellToWorldLocation(cell) - LaserLocation)) / 2;
        double distance = DistanceToCell(cell, perception->GetLaserLocation());
        double p_o = ComputeProbability(distance, 0, 0.98);
        double p_e = 1 - p_o;
        double P_O = occupancyGrid.GetCellValue(cell);
        double P_E = (1 - P_O) ;
        double p = (p_o * P_O) / ((p_o * P_O) + (p_e * P_E)); //p(Occupied | s)
        occupancyGrid.UpdateCell(cell.x, cell.y, p);
    }

    //Cells in regionII (possible free spaces)
    for (const auto &cell : RegionII) {
        double alpha = glm::angle(LaserVector, glm::normalize(CellToWorldLocation(cell) - LaserLocation))  / 2;
        //std::cout << "alpha: " << alpha << std::endl;
        double distance = DistanceToCell(cell, perception->GetLaserLocation());
        double p_e = ComputeProbability(distance, 0, 0.99);
        double p_o = 1 - p_e;
        double P_O = occupancyGrid.GetCellValue(cell);
        double P_E = 1 - P_O;
        double p = (p_e * P_E) / ((p_e * P_E) + (p_o * P_O)); //p(Empty | s)

        occupancyGrid.UpdateCell(cell.x, cell.y, 1-p); //1- cause 0 represent sure that empty
    }
}

void Cartographer::Update() {
    if (!perception) {
        return;
    }

    const std::vector<double> &Distances = perception->GetEchoDistances();


    if (perception->LastEchoTimestamp() != lastTimestamp) {
        lastTimestamp = perception->LastEchoTimestamp();
    } else {
        return;
    }


    std::vector<glm::ivec2> PossibleRegionCells;
    GetCellsInRobotRange(40, PossibleRegionCells);
    for (int i = perception->GetStartLaserIndex(); i <= perception->GetEndLaserIndex(); ++i) { //Distances.size()
        //for (int i = 133; i <= 137 ; ++i) { //Distances.size()
        double GlobalLaserHeading = perception->GetLaserHeading(i);
        const glm::dvec3 LaserVector = Perception::AngleToVector(GlobalLaserHeading);
        HandleEcho(Distances[i], LaserVector, PossibleRegionCells);
    }
}

glm::dvec3 Cartographer::RobotForwardVector() const {
    return perception->GetRobotForwardVector();
}

glm::dvec3 Cartographer::RobotLocation() const {
    return perception->GetLaserLocation();
}


