#pragma once

#include <external/glm/vec3.hpp>
#include "Grid.h"
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
    void Update(); // call each loop

    void SetPreception(const std::shared_ptr<Perception> &perception);

    /* add cell to be considered "known" ie we have seen this cell */
    void AddKnownCell(const glm::ivec2 &cell);


    /* Read the the global position of a laser hit */
    glm::dvec3 ComputeWorldHitLocation(double Distance, const glm::dvec3 &LaserPosition, double GlobalLaserHeading) const;
    /* Read the the local position of a laser hit */
    glm::dvec3 ComputeLocalHitLocation(double Distance, double GlobalLaserHeading) const;

    glm::dvec3 WorldLaserDirection(double GlobalLaserHeading) const;    // Convert angle heading to a vector
    const std::vector<std::vector<double>> &GetProbablityGrid() const; // Regference to occupancy grid

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
    glm::dvec3 RobotForwardVector() const;

    /* Get cells next to cell,include diagonal */
    std::vector<glm::ivec2> GetAdjacent(glm::ivec2 cell) const;
    std::vector<glm::ivec2> GetEmptyAdjacent(glm::ivec2 cell) const;

    /* Get cells next to cell, exclude diagonal */
    std::vector<glm::ivec2> GetNeighbors(glm::ivec2 cell) const;
    std::vector<glm::ivec2> GetNeighborsLessThan(glm::ivec2 cell, float p) const;

    double GetProbabilityEmpty(glm::ivec2 cell) const;

    bool IsUnknown(glm::ivec2 cell) const;

    glm::dvec3 RobotLocation() const;



private:
    double Beta = 0.00872665;   // half main lobe angle(in radians)
    double Region1Range = 2;     // the width of regionI

    Grid occupancyGrid;         // Contains probabilities, empty = 0, occupied = 1
    Grid exploredGrid;          // Contains explored cells, unexplored = 0, explored = 1
    std::shared_ptr<Perception> perception; // What the robot perceive

    int lastTimestamp;          // Last handle time from perception
    double cellSize;            // size of a cell

    double GetCellHeight() const;
    double GetCellWidth() const;

    double DistanceToCell(glm::ivec2 cell, glm::dvec3 position) const;
    inline double HalfCellSize() const { return cellSize / 2.0; }
    bool IsOutside(glm::dvec3 Point, glm::dvec3 Normal) const;


    /* Handle a single laser reading */
    void HandleEcho(double distanceToHit, const glm::dvec3 &LaserVector, const std::vector<glm::ivec2> &CellsInRange);

    /* Get all cells within range disance*/
    void GetCellsInRange(const glm::dvec3 &worldlocation, double range, std::vector<glm::ivec2> &outCells) const;
    void GetCellsInRange(const glm::ivec2 &cell, double range, std::vector<glm::ivec2> &outCells) const;


    /* Get regionI and regionII */
    void GetRegions(const glm::dvec3 LaserLocation, const glm::dvec3 WorldHitLocation, const glm::dvec3 &LaserVector, const std::vector<glm::ivec2> &CellsInRange,
                    std::vector<glm::ivec2> &OutRegionI, std::vector<glm::ivec2> &OutRegionII);

    /* Is the cell within the angle of a laser */
    bool IsCellWithinBoundry(glm::ivec2 cell, glm::dvec3 LaserVector) const;

    /* Is the a square given by CellLocalLocation in front of RobotForward and between LineA and Line B*/
    bool IsBoxInside(glm::dvec3 LineA, glm::dvec3 LineB, glm::dvec3 CellLocalLocation) const;

    /* Compute probability p(s|H) */
    double ComputeProbability(double distance, double alpha, double pMax) const;
};


