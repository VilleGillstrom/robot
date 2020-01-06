#pragma once

#include <external/glm/vec3.hpp>
#include "Grid.h"
#include "RobotTypes.h"
#include "SonarModel.h"
#include "Perception.h"

#include <glm/glm.hpp>
#include <iostream>
#include <cmath>
#include <glm/gtx/string_cast.hpp>
#include <glm/gtx/vector_angle.hpp>
#include <QtWidgets/QDialog>

class Cartographer {


public:
    Cartographer(int cellsize, int xmin, int ymin, int xmax, int ymax);

    void SetPreception(const std::shared_ptr<Perception> &perception );

    void Update() {
        if (!perception) {
            return;
        }

        const std::vector<double> &Distances = perception->GetEchoDistances();


        double StartAngle = perception->GetStartAngle();
        double AngleIncrement = perception->GetAngleIncrement();
        double Heading = perception->GetHeading();


        if (perception->LastEchoTimestamp() != lastTimestamp) {
            lastTimestamp = perception->LastEchoTimestamp();
        } else {
            return;
        }


        std::vector<glm::ivec2> PossibleRegionCells;
        GetCellsInRange(perception->GetLaserMaxRange(),PossibleRegionCells);
        for (int i = perception->GetStartLaserIndex(); i <=perception->GetEndLaserIndex(); ++i) { //Distances.size()
        //for (int i = 133; i <= 137 ; ++i) { //Distances.size()
            double GlobalLaserHeading = perception->GetLaserHeading(i);
            const glm::dvec3 LaserVector = Perception::AngleToVector(GlobalLaserHeading);
            HandleEcho(Distances[i], LaserVector, PossibleRegionCells);
        }
    }



    void HandleEcho(double distance, const glm::dvec3& LaserVector, const std::vector<glm::ivec2>& CellsInRange) {
        const glm::dvec3 &LaserLocation = perception->GetLaserLocation();
        const glm::dvec3 WorldHitLocation = LaserLocation + LaserVector * distance;

        std::vector<glm::ivec2> RegionI;
        std::vector<glm::ivec2> RegionII;
        GetRegions(LaserLocation, WorldHitLocation, LaserVector, CellsInRange, RegionI, RegionII);

        for (const glm::ivec2 &cell : CellsInRange) {
            //occupancyGrid.UpdateCell(cell, 0.0);
        }

        //Cells in regionI (possible walls)
        for (const auto &cell : RegionI) {
            double p = ComputeProbability(DistanceToCell(cell, perception->GetLaserLocation()));
            std::cout << p << std::endl;
            double newProbablity = p * occupancyGrid.GetCellValue(cell) ;
            occupancyGrid.UpdateCell(cell.x, cell.y, newProbablity);
        }

        //Cells in regionII (possible free spaces)
        for (const auto &cell : RegionII) {
            double p = ComputeProbability(DistanceToCell(cell, perception->GetLaserLocation()));
            double newProbablity = (1 - p) * occupancyGrid.GetCellValue(cell) ;
            occupancyGrid.UpdateCell(cell.x, cell.y, newProbablity);
        }
    }


    void AddKnownCell(const glm::ivec2 &cell);

    void GetRegions(const glm::dvec3 LaserLocation, const glm::dvec3 WorldHitLocation, const glm::dvec3 &LaserVector,
                    const std::vector<glm::ivec2> &CellsInRange,
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

            //Is cell within angle?
            if (isWithin) {
                double CellDistanceFromHit = glm::length(cellLocalLocation - (WorldHitLocation - LaserLocation));
                if (CellDistanceFromHit < 2.0) {
                    OutRegionI.push_back(cell);
                } else {
                    OutRegionII.push_back(cell);
                }
                AddKnownCell(cell);
            }
        }
    }

    bool IsCellWithinBoundry(glm::ivec2 cell, glm::dvec3 LaserVector) const;

    /** Is the a square given by CellLocalLocation in front of RobotForward and between LineA and Line B*/
    bool IsBoxInside(glm::dvec3 LineA, glm::dvec3 LineB, glm::dvec3 CellLocalLocation) const;


    // The the ponit in a cell that is closest along a normal
    glm::dvec3 GetVertexN(const glm::dvec3 &normal, glm::dvec3 cll) const;

    // The the ponti in a cell that is furthes along a normal
    glm::dvec3 GetVertexP(const glm::vec3 &normal, glm::dvec3 cll) const;





    double ComputeProbability(double distance) const {
        double R = MaxDistance; // Max distance
        double r = distance;    // distance to hit
        double Pmax = 0.98;     // max probabiliy
        double a = 0;           // tolearn

        double Rratio = (R - r) / R;
        double AngleRatio = (Beta - a) / Beta;
        return ((Rratio + AngleRatio) / 2) * Pmax;
    }

    /** Read the the global position of a laser hit */
    glm::dvec3
    ComputeWorldHitLocation(double Distance, const glm::dvec3 &LaserPosition, double GlobalLaserHeading) const;

    glm::dvec3 ComputeLocalHitLocation(double Distance, double GlobalLaserHeading) const;

    glm::dvec3 WorldLaserDirection(double GlobalLaserHeading) const;


    const std::vector<std::vector<double>> &GetProbablityGrid() const;

    int GetXMin() const;
    int GetYMin() const;
    int MapWidth() const;
    int MapHeight() const;

    glm::dvec3 CellToWorldLocation(const glm::ivec2 &cell) const;
    glm::dvec3 CellToWorldLocation(int row, int col) const;
    glm::ivec2 WorldLocationToCell(double x, double y) const;
    glm::ivec2 WorldLocationToCell(const glm::dvec3 &WorldLocation) const;
    void GetCellsInRange(double range, std::vector<glm::ivec2> &outCells);
    double CellSize() const {return cellSize;};

    /* Get cells next to cell,include diagonal */
    std::vector<glm::ivec2> GetAdjacent(glm::ivec2 cell) const;
    /* Get cells next to cell, exclude diagonal */

    std::vector<glm::ivec2> GetNeighbors(glm::ivec2 cell) const;
    double GetProbabilityEmpty(glm::ivec2 cell) const;
    bool IsUnknown(glm::ivec2 cell) const;
    glm::dvec3 RobotLocation() const {
        return perception->GetLaserLocation();
    }

private:
    double MaxDistance = 40.0;
    double Beta = 0.00872665;         // half main lobe angle(in radians)

    SonarModel sonarModel;

    Grid occupancyGrid;
    Grid exploredGrid;
    std::shared_ptr<Perception> perception;

    double GetCellHeight() const;
    double GetCellWidth() const;

    inline glm::dvec3 TopRightCellCorner(const glm::dvec3 Location) const {
        return {Location.x + HalfCellSize(), Location.y + HalfCellSize(), Location.z};
    }
    inline glm::dvec3 TopLeftCellCorner(const glm::dvec3 Location) const {
        return {Location.x - HalfCellSize(), Location.y + HalfCellSize(), Location.z};
    }
    inline glm::dvec3 BotRightCellCorner(const glm::dvec3 Location) const {
        return {Location.x + HalfCellSize(), Location.y - HalfCellSize(), Location.z};
    }
    inline glm::dvec3 BotLeftCellCorner(const glm::dvec3 Location) const {
        return {Location.x - HalfCellSize(), Location.y - HalfCellSize(), Location.z};
    }


    double DistanceToCell(glm::ivec2 cell, glm::dvec3 position) const;
    inline double HalfCellSize() const { return cellSize / 2.0; }
    bool IsOutside(glm::dvec3 Point, glm::dvec3 Normal) const;

    int lastTimestamp;
    int cellSize;
};


