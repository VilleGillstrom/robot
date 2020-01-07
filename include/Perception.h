#pragma once

#include <memory>
#include "RobotCommunicationMRDS.h"
#include <glm/glm.hpp>
#include <external/glm/gtx/rotate_vector.hpp>

/**
 * Perception of the world. Should be used to get sensor data.
 */
class Perception {

public:
    Perception(const std::shared_ptr<RobotCommunicationMRDS> &Communicator);



    void ReadSensors();
    glm::dvec3 GetLaserLocation() const;
    glm::dvec3 GetPosition() const;
    glm::dquat GetOrientation() const;
    const std::vector<double> &GetEchoDistances() const;

    double GetStartAngle() const;
    double GetEndAngle() const;

    double GetAngleIncrement() const;
    double GetHeading() const;
    double GetLaserMaxRange() const;

    glm::dvec3 GetRobotForwardVector() const;

    static glm::dvec3 AngleToVector(double heading);

    /* Get the world vector of a laser echo at index */
    glm::dvec3 GetLaserWorldVector(int LaserIndex) const;
    double GetLaserHeading(int LaserIndex) const;
    double GetLaserSpan() const;
    glm::dvec3 GetLaserLeftEnd() const;
    glm::dvec3 GetLaserRightEnd() const;
    int GetStartLaserIndex() const;
    int GetEndLaserIndex() const;
    int LastEchoTimestamp() const;


private:
    int StartLaserIndex = 135-135;
    int EndLaserIndex = 135+135;

    std::shared_ptr<RobotCommunicationMRDS> communicator;
    /* Last Echoes*/
    RobotCommunicationMRDS::laser_echos echoes;
    /* Last localization */
    RobotCommunicationMRDS::robot_localization localization{};
    /* laser properties */
    RobotCommunicationMRDS::laser_properties laserProperties{};


};
