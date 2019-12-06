//
// Created by twentyletters on 2019-12-05.
//

#include "include/Robot.h"
#include <glm/gtx/string_cast.hpp>

Robot::Robot(const std::shared_ptr<RobotCommunicationMRDS> &RobotCommunicator)  : perception(RobotCommunicator), cartoGrapher(-100, -100, 100, 100) {
    this->RobotCommunicator = RobotCommunicator;
    ReadProperties();
}

void Robot::ReadProperties() {
    if (RobotCommunicator) {
        RobotCommunicator->ReadProperties();
        auto robotProperties = RobotCommunicator->GetProperties();
        EndAngle = robotProperties.EndAngle;
        StartAngle = robotProperties.StartAngle;
        AngleIncrement = robotProperties.AngleIncrement;
    }
}

void Robot::SetCommunicator(const std::shared_ptr<RobotCommunicationMRDS> &NewRobotCommunicator) {
    this->RobotCommunicator = NewRobotCommunicator;
}


double Robot::QuatToHeadingAngle(const glm::quat &Orientation) const {
    return glm::eulerAngles(Orientation).z;
}

glm::dvec3 Robot::GetPosition() const {
    return perception.GetPosition();
}

const Cartographer &Robot::GetCartographer() const {
    return cartoGrapher;
}

void Robot::UpdateMap() {


    const glm::quat &orientation = perception.GetOrientation();
    const std::vector<double>& Distances = perception.GetEchoDistances();
    const glm::dvec3 &position = perception.GetPosition();

    double Heading = QuatToHeadingAngle(orientation);
    for(int i = 0; i < Distances.size(); ++i) { //echoes.values.size()
        auto distance = Distances[i];
        double LocalLaserHeading = StartAngle + AngleIncrement * i;
        double GlobalLaserHeading =  Heading + LocalLaserHeading;
        cartoGrapher.HandleEcho(distance, position, GlobalLaserHeading);
    }
}

