#pragma once

#include <external/glm/vec3.hpp>
#include "OccupancyGrid.h"
#include "RobotTypes.h"
#include "SonarModel.h"
#include <glm/gtx/rotate_vector.hpp>
#include <glm/glm.hpp>
#include <iostream>
#include <cmath>
#include <glm/gtx/string_cast.hpp>



class Cartographer {
    OccupancyGrid occupancyGrid;
public:
    Cartographer(int xmin, int ymin, int xmax, int ymax) : occupancyGrid(xmin,  ymin,  xmax,  ymax)  {

    }

    void HandleEcho(double distance, glm::dvec3 LaserPosition, double globalHeading) {
        ComputeProbability(distance);

        const glm::vec3 HitWorldPosition = ComputePosition(distance, LaserPosition, globalHeading);
        // if sonar hit something
        if(distance < 40) {
            //GetRegion2(HitWorldPosition, globalHeading )
        }

        if(distance < 30) {
            occupancyGrid.UpdatePosition(HitWorldPosition.x, HitWorldPosition.y, 1.0);
        }
    }


    /** Read the the global position of a laser hit */
    glm::vec3 ComputePosition(double Distance, const glm::dvec3 &LaserPosition, double GlobalLaserHeading) const {
        glm::dvec3 north(0.0, 1.0, 0.0);
        glm::dvec3 HeadingVector = glm::rotateZ(north, GlobalLaserHeading);
        glm::vec3 HitPosition = LaserPosition + HeadingVector * Distance;
        return HitPosition;
    }

    double ComputeProbability(double distance) const {
        double R = MaxDistance; // Max distance
        double B = 0.00872665;         // half main lobe angle(in radian)
        double r = distance;    // distance to hit
        double Pmax = 0.98;     // max probabiliy
        double a = 0;           // tolearn

        double PsOccupied  = (((R - r ) / R) - ((B - a) / B) / 2) * Pmax;
        double PsEmpty = 1 -PsOccupied;
        return PsOccupied;
    }

    const std::vector<std::vector<double>>& GetProbablityGrid() const;
    int GetXMin() const;
    int GetYMin() const;

private:
    double MaxDistance = 40.0;

    SonarModel sonarModel;
};


