#pragma once

#include <memory>
#include "RobotCommunicationMRDS.h"
#include <glm/glm.hpp>
#include <external/glm/gtx/rotate_vector.hpp>

class Perception {

public:
    Perception(const std::shared_ptr<RobotCommunicationMRDS> &Communicator) : communicator(Communicator) {
        if (Communicator) {
            Communicator->ReadProperties();
            laserProperties = Communicator->GetProperties();
        }
    }

    void ReadSensors() {
        communicator->ReadSensors();
        echoes = communicator->GetEchoes();
        localization = communicator->GetLocalization();
        //std::cout << "echoes stamp: " << echoes.timestamp  << ", loc stamp: " << localization.timestamp << std::endl;
    }

    glm::dvec3 GetLaserPosition() const {
        return GetPosition() + glm::rotateZ(laserProperties.LaserOffset, GetHeading());

    }

    glm::dvec3 GetPosition() const {
        return glm::dvec3(localization.position);

    }

    glm::dquat GetOrientation() const { return localization.orientation; }

    const std::vector<double> &GetEchoDistances() const { return echoes.distance; }

    double GetStartAngle() const { return laserProperties.StartAngle; }

    double GetEndAngle() const { return laserProperties.EndAngle; }

    double GetAngleIncrement() const { return laserProperties.AngleIncrement; }

    double GetHeading() const { return glm::eulerAngles(GetOrientation()).z; }

    glm::dvec3 GetHeadingVector() const {
        return glm::normalize(AngleToVector(GetHeading()));
    }

    static glm::dvec3 AngleToVector(double heading) {
        return glm::rotateZ(glm::dvec3(1.0, 0.0, 0.0), heading);
    }

    glm::dvec3 GetLaserWorldVector(int LaserIndex) const {
        return AngleToVector(GetLaserHeading(LaserIndex));
    }

    double GetLaserHeading(int LaserIndex) const {
        return GetHeading() + laserProperties.StartAngle + LaserIndex * laserProperties.AngleIncrement;
    }


    glm::dvec3 GetLaserLeftEnd() const {
        return glm::rotateZ(GetHeadingVector(), GetStartAngle());
    }

    glm::dvec3 GetLaserRightEnd() const {
        return glm::rotateZ(GetHeadingVector(), GetEndAngle());
    }
private:
    std::shared_ptr<RobotCommunicationMRDS> communicator;
    /* Last Echoes*/
    RobotCommunicationMRDS::laser_echos echoes;
    /* Last localization */
    RobotCommunicationMRDS::robot_localization localization;
    /* laser properties */
    RobotCommunicationMRDS::laser_properties laserProperties;
};
