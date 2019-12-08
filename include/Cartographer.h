#pragma once

#include <external/glm/vec3.hpp>
#include "OccupancyGrid.h"
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
    OccupancyGrid occupancyGrid;
    std::shared_ptr<Perception> perception;

public:
    Cartographer(int xmin, int ymin, int xmax, int ymax) : occupancyGrid(xmin, ymin, xmax, ymax) {

    }

    void SetPreception(const std::shared_ptr<Perception> &perception);

    void Update() {
        if (!perception) {
            return;
        }

        const std::vector<double> &Distances = perception->GetEchoDistances();
        const glm::dvec3 &position = perception->GetLaserPosition();

        double StartAngle = perception->GetStartAngle();
        double AngleIncrement = perception->GetAngleIncrement();
        double Heading = perception->GetHeading();


        if (perception->LastEchoTimestamp() != lastTimestamp) {
            lastTimestamp = perception->LastEchoTimestamp();
        } else {
            return;
        }


        for (int i = perception->GetStartLaserIndex(); i <= perception->GetEndLaserIndex(); ++i) { //Distances.size()
            double GlobalLaserHeading = perception->GetLaserHeading(i);
            HandleEcho(Distances[i], position, GlobalLaserHeading);
        }
    }


    void GetRegions(const glm::dvec3 LaserLocation, const glm::dvec3 WorldHitLocation, double LaserWorldHeading,
                    double Beta, std::vector<glm::ivec2> &RegionI, std::vector<glm::ivec2> &RegionII) {

        glm::dvec3 LaserVector = Perception::AngleToVector(LaserWorldHeading);


        std::vector<glm::ivec2> RegionICells;
        std::vector<glm::ivec2> RegionIICells;

        auto grid = occupancyGrid.GetGrid();
        for (int c = 0; c < occupancyGrid.Columns(); ++c) {
            for (int r = 0; r < occupancyGrid.Rows(); ++r) {
                glm::dvec3 cellWorldLocation = CellToWorldLocation(r, c);
                glm::dvec3 cellLocalLocation = cellWorldLocation - LaserLocation;

                double CellDistance = glm::length(cellLocalLocation);
                double RobotDistanceToHit = glm::length(LaserLocation - WorldHitLocation);
                
                if (CellDistance > RobotDistanceToHit) { // if cell is beyond the hit skip cell
                    continue;
                }


                bool isWithin = IsCellWithinBoundry({r, c}, LaserVector, Beta);
                //Is cell within angle?
                if (isWithin) {
                    double CellDistanceFromHit = glm::length(cellLocalLocation - (LaserLocation - WorldHitLocation));
                    if (CellDistanceFromHit < 4.0) {
                        RegionICells.push_back({r, c});
                    } else {
                        RegionIICells.push_back({r, c});

                    }

                }

            }
        }
        RegionI = RegionICells;
        RegionII = RegionIICells;
    }

    bool IsCellWithinBoundry(glm::ivec2 cell, glm::dvec3 LaserVector, double Beta) const;

    /** Is the a square given by CellLocalLocation in front of RobotForward and between LineA and Line B*/
    bool IsBoxInside(glm::dvec3 RobotForward, glm::dvec3 LineA, glm::dvec3 LineB, glm::dvec3 CellLocalLocation) const;


    // The the ponit in a cell that is closest along a normal
    glm::dvec3 GetVertexN(const glm::dvec3 &normal, glm::dvec3 cll) const;

    // The the ponti in a cell that is furthes along a normal
    glm::dvec3 GetVertexP(const glm::vec3 &normal, glm::dvec3 cll) const;


    glm::dvec3 CellToWorldLocation(int row, int col) const;

    glm::dvec3 CellToWorldLocation(glm::ivec2 cell) const;

    glm::ivec2 WorldLocationToCell(double x, double y) const;

    glm::ivec2 WorldLocationToCell(const glm::dvec3 &WorldLocation) const;

    void HandleEcho(double distance, glm::dvec3 LaserLocation, double WorldLaserHeading) {
        ComputeProbability(distance);
        //std::cerr << glm::to_string(LaserLocation) << std::endl;

        const glm::dvec3 WorldHitLocation = ComputeWorldHitLocation(distance, LaserLocation, WorldLaserHeading);
        // if sonar hit something
        if (distance < 10000) {
            std::vector<glm::ivec2> RegionI;
            std::vector<glm::ivec2> RegionII;
            GetRegions(LaserLocation, WorldHitLocation, WorldLaserHeading, B, RegionI, RegionII);
            //std::cerr << asd.size() << std::endl;

            for (const auto &cell : RegionII) {
                //occupancyGrid.UpdateCell(item.x, item.y, 0.0);
                occupancyGrid.UpdateCell(cell.x, cell.y, 0.0);
            }
            for (const auto &cell : RegionI) {
                //occupancyGrid.UpdateCell(item.x, item.y, 0.0);
                occupancyGrid.UpdateCell(cell.x, cell.y, 1);
            }

        }

        if (distance < 40) {
            //std::cout << glm::to_string(WorldHitLocation) << std::endl;
            occupancyGrid.UpdateLocation(WorldHitLocation.x, WorldHitLocation.y, 1.0);
        }
    }


    /** Read the the global position of a laser hit */
    glm::dvec3
    ComputeWorldHitLocation(double Distance, const glm::dvec3 &LaserPosition, double GlobalLaserHeading) const {
        glm::dvec3 LocalHitLocation = ComputeLocalHitLocation(Distance, GlobalLaserHeading);
        glm::dvec3 GlobalHitLocation = LaserPosition + LocalHitLocation;
        return GlobalHitLocation;
    }

    glm::dvec3 ComputeLocalHitLocation(double Distance, double GlobalLaserHeading) const {
        glm::dvec3 HeadingVector = WorldLaserDirection(GlobalLaserHeading);
        glm::dvec3 LocalHitPosition = HeadingVector * Distance;
        return LocalHitPosition;
    }

    glm::dvec3 WorldLaserDirection(double GlobalLaserHeading) const {
        glm::dvec3 xvec(1, 0, 0.0);
        glm::dvec3 HeadingVector = glm::rotateZ(xvec, GlobalLaserHeading);
        return HeadingVector;
    }

    double ComputeProbability(double distance) const {
        double R = MaxDistance; // Max distance
        double r = distance;    // distance to hit
        double Pmax = 0.98;     // max probabiliy
        double a = 0;           // tolearn

        double PsOccupied = (((R - r) / R) - ((B - a) / B) / 2) * Pmax;
        double PsEmpty = 1 - PsOccupied;
        return PsOccupied;
    }

    const std::vector<std::vector<double>> &GetProbablityGrid() const;

    int GetXMin() const;

    int GetYMin() const;

private:
    double MaxDistance = 40.0;
    SonarModel sonarModel;

    double B = 0.00872665;         // half main lobe angle(in radians)


    double GetCellHeight() const;

    double GetCellWidth() const;

    inline glm::dvec3 TopRightCellCorner(const glm::dvec3 Location) const {
        return {Location.x + 0.5, Location.y + 0.5, Location.z};
    }

    inline glm::dvec3 TopLeftCellCorner(const glm::dvec3 Location) const {
        return {Location.x - 0.5, Location.y + 0.5, Location.z};
    }

    inline glm::dvec3 BotRightCellCorner(const glm::dvec3 Location) const {
        return {Location.x + 0.5, Location.y - 0.5, Location.z};
    }

    inline glm::dvec3 BotLeftCellCorner(const glm::dvec3 Location) const {
        return {Location.x - 0.5, Location.y - 0.5, Location.z};
    }

    bool IsOutside(glm::dvec3 Point, glm::dvec3 Normal) const;

    int lastTimestamp;
};


