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
    }

    glm::dvec3 GetLaserLocation() const {
        return GetPosition() + glm::rotateZ(laserProperties.LaserOffset, GetHeading());

    }

    glm::dvec3 GetPosition() const {
        return glm::dvec3(localization.position);

    }

    glm::dquat GetOrientation() const { return localization.orientation; }

    const std::vector<double> &GetEchoDistances() const { return echoes.distance; }

    double GetStartAngle() const {
        return laserProperties.StartAngle + StartLaserIndex * GetAngleIncrement();;
    }

    double GetEndAngle() const { return laserProperties.StartAngle + EndLaserIndex *GetAngleIncrement(); }

    double GetAngleIncrement() const { return laserProperties.AngleIncrement; }

    double GetHeading() const { return glm::eulerAngles(GetOrientation()).z; }

    double GetLaserMaxRange() const { return 40.0; }


    glm::dvec3 GetRobotForwardVector() const {
        return glm::normalize(AngleToVector(GetHeading()));
    }

    static glm::dvec3 AngleToVector(double heading) {
        return glm::normalize(glm::rotateZ(glm::dvec3(1.0, 0.0, 0.0), heading));
    }

    glm::dvec3 GetLaserWorldVector(int LaserIndex) const {
        return AngleToVector(GetLaserHeading(LaserIndex));
    }

    double GetLaserHeading(int LaserIndex) const {
        return GetHeading() + laserProperties.StartAngle + LaserIndex * laserProperties.AngleIncrement;
    }

    double GetLaserSpan() const {
        return (GetEndLaserIndex()-GetStartLaserIndex() )*laserProperties.AngleIncrement;
    }

    glm::dvec3 GetLaserLeftEnd() const {
        const glm::dvec3 &LaserHeading = glm::rotateZ(GetRobotForwardVector(), GetStartAngle() );
        return GetLaserLocation() + (LaserHeading * 40.0) ;
    }

    glm::dvec3 GetLaserRightEnd() const {
        const glm::dvec3 LaserHeading = glm::rotateZ(GetRobotForwardVector(), GetStartAngle() );
        return GetLaserLocation() + (LaserHeading * 40.0) ;

    }

    int GetStartLaserIndex() const {
        return StartLaserIndex;
    }

    int GetEndLaserIndex() const {
        return EndLaserIndex;
    }

    int LastEchoTimestamp() const {
        return echoes.timestamp;
    }
private:
    int StartLaserIndex = 135-135;
    int EndLaserIndex = 135+135;

    std::shared_ptr<RobotCommunicationMRDS> communicator;
    /* Last Echoes*/
    RobotCommunicationMRDS::laser_echos echoes;
    /* Last localization */
    RobotCommunicationMRDS::robot_localization localization;
    /* laser properties */
    RobotCommunicationMRDS::laser_properties laserProperties;

};
