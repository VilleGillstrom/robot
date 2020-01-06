//
// Created by twentyletters on 2019-12-06.
//

#include "include/Perception.h"

double Perception::GetLaserHeading(int LaserIndex) const {
    return GetHeading() + laserProperties.StartAngle + LaserIndex * laserProperties.AngleIncrement;
}

double Perception::GetLaserSpan() const {
    return (GetEndLaserIndex()-GetStartLaserIndex() )*laserProperties.AngleIncrement;
}

Perception::Perception(const std::shared_ptr<RobotCommunicationMRDS> &Communicator) : communicator(Communicator) {
    if (Communicator) {
        Communicator->ReadProperties();
        laserProperties = Communicator->GetProperties();
    }
}

void Perception::ReadSensors() {
    communicator->ReadRobot();
    echoes = communicator->GetEchoes();
    localization = communicator->GetLocalization();
}

glm::dvec3 Perception::GetLaserLocation() const {
    return GetPosition() + glm::rotateZ(laserProperties.LaserOffset, GetHeading());

}

glm::dvec3 Perception::GetPosition() const {
    return glm::dvec3(localization.position);

}

glm::dquat Perception::GetOrientation() const { return localization.orientation; }

const std::vector<double> &Perception::GetEchoDistances() const { return echoes.distance; }

double Perception::GetStartAngle() const {
    return laserProperties.StartAngle + StartLaserIndex * GetAngleIncrement();;
}

double Perception::GetEndAngle() const { return laserProperties.StartAngle + EndLaserIndex *GetAngleIncrement(); }

double Perception::GetAngleIncrement() const { return laserProperties.AngleIncrement; }

double Perception::GetHeading() const { return glm::eulerAngles(GetOrientation()).z; }

double Perception::GetLaserMaxRange() const { return 40.0; }

glm::dvec3 Perception::GetRobotForwardVector() const {
    return glm::normalize(AngleToVector(GetHeading()));
}

glm::dvec3 Perception::AngleToVector(double heading) {
    return glm::normalize(glm::rotateZ(glm::dvec3(1.0, 0.0, 0.0), heading));
}

glm::dvec3 Perception::GetLaserWorldVector(int LaserIndex) const {
    return AngleToVector(GetLaserHeading(LaserIndex));
}

glm::dvec3 Perception::GetLaserLeftEnd() const {
    const glm::dvec3 &LaserHeading = glm::rotateZ(GetRobotForwardVector(), GetStartAngle() );
    return GetLaserLocation() + (LaserHeading * 40.0) ;
}

glm::dvec3 Perception::GetLaserRightEnd() const {
    const glm::dvec3 LaserHeading = glm::rotateZ(GetRobotForwardVector(), GetStartAngle() );
    return GetLaserLocation() + (LaserHeading * 40.0) ;

}

int Perception::GetStartLaserIndex() const {
    return StartLaserIndex;
}

int Perception::GetEndLaserIndex() const {
    return EndLaserIndex;
}

int Perception::LastEchoTimestamp() const {
    return echoes.timestamp;
}
