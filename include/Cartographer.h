#pragma once

#include <external/glm/vec3.hpp>
#include "Grid.h"
#include "RobotTypes.h"
#include "Perception.h"

#include <glm/glm.hpp>
#include <iostream>
#include <cmath>
#include <glm/gtx/string_cast.hpp>
#include <glm/gtx/vector_angle.hpp>
#include <QtWidgets/QDialog>
#include<algorithm>

class Cartographer {


public:
    Cartographer(double cellsize, int xmin, int ymin, int xmax, int ymax);

    void SetPreception(const std::shared_ptr<Perception> &perception);

    void Update();
    void HandleEcho(double distanceToHit, const glm::dvec3 &LaserVector, const std::vector<glm::ivec2> &CellsInRange);
    void AddKnownCell(const glm::ivec2 &cell);

    void GetRegions(const glm::dvec3 LaserLocation, const glm::dvec3 WorldHitLocation, const glm::dvec3 &LaserVector, const std::vector<glm::ivec2> &CellsInRange,
                    std::vector<glm::ivec2> &OutRegionI, std::vector<glm::ivec2> &OutRegionII);

    bool IsCellWithinBoundry(glm::ivec2 cell, glm::dvec3 LaserVector) const;

    /** Is the a square given by CellLocalLocation in front of RobotForward and between LineA and Line B*/
    bool IsBoxInside(glm::dvec3 LineA, glm::dvec3 LineB, glm::dvec3 CellLocalLocation) const;

    double ComputeProbability(double distance, double alpha, double pMax) const;


    /** Read the the global position of a laser hit */
    glm::dvec3  ComputeWorldHitLocation(double Distance, const glm::dvec3 &LaserPosition, double GlobalLaserHeading) const;
    glm::dvec3 ComputeLocalHitLocation(double Distance, double GlobalLaserHeading) const;
    glm::dvec3 WorldLaserDirection(double GlobalLaserHeading) const;
    const std::vector<std::vector<double>> &GetProbablityGrid() const;
    int GetXMin() const;
    int GetYMin() const;
    int GetYMax() const;
    int GetXMax() const;
    int MapWidth() const;
    int MapHeight() const;

    glm::dvec3 CellToWorldLocation(const glm::ivec2 &cell) const;   /* Convert a cell to world location t */
    glm::dvec3 CellToWorldLocation(int row, int col) const; /* Convert a cell to world location t */
    glm::ivec2 WorldLocationToCell(double x, double y) const;   /* Convert a world location to cell  */
    glm::ivec2 WorldLocationToCell(const glm::dvec3 &WorldLocation) const; /* Convert a world location to cell  */
    void GetCellsInRobotRange(double range, std::vector<glm::ivec2> &outCells); /* What cells are in range of the robot */

    double CellSize() const { return cellSize; };

    glm::dvec3 RobotForwardVector() const {
        return perception->GetRobotForwardVector();
    }

    /* Get cells next to cell,include diagonal */
    std::vector<glm::ivec2> GetAdjacent(glm::ivec2 cell) const;
    std::vector<glm::ivec2> GetEmptyAdjacent(glm::ivec2 cell) const;

    /* Get cells next to cell, exclude diagonal */
    std::vector<glm::ivec2> GetNeighbors(glm::ivec2 cell) const;
    /* Get neighbors with less than p in occupancy grid */
    std::vector<glm::ivec2> GetNeighborsLessThan(glm::ivec2 cell, float p) const;

    double GetProbabilityEmpty(glm::ivec2 cell) const;

    bool IsUnknown(glm::ivec2 cell) const;

    glm::dvec3 RobotLocation() const {
        return perception->GetLaserLocation();
    }

    glm::ivec2 RobotCell() const {
        return WorldLocationToCell(RobotLocation());
    }

private:
    double MaxDistance = 40.0;
    double Beta = 0.00872665;         // half main lobe angle(in radians)
    double Region1Range = 2;

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
    double cellSize;


    void GetCellsInRange(const glm::dvec3 &worldlocation, double range, std::vector<glm::ivec2> &outCells) const;

    void GetCellsInRange(const glm::ivec2 &cell, double range, std::vector<glm::ivec2> &outCells) const;

};


