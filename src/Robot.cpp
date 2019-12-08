//
// Created by twentyletters on 2019-12-05.
//

#include "include/Robot.h"
#include <glm/gtx/string_cast.hpp>

Robot::Robot(const std::shared_ptr<RobotCommunicationMRDS> &RobotCommunicator)  :  cartoGrapher(1, -100, -100, 100, 100) {
    this->RobotCommunicator = RobotCommunicator;
    perception = std::make_shared<Perception>(RobotCommunicator);
    cartoGrapher.SetPreception(perception);
}



void Robot::SetCommunicator(const std::shared_ptr<RobotCommunicationMRDS> &NewRobotCommunicator) {
    this->RobotCommunicator = NewRobotCommunicator;
}


double Robot::QuatToHeadingAngle(const glm::quat &Orientation) const {
    return glm::eulerAngles(Orientation).z;
}

glm::dvec3 Robot::GetPosition() const {
    return perception->GetLaserLocation();
}

Cartographer &Robot::GetCartographer()  {
    return cartoGrapher;
}

std::shared_ptr<Perception> Robot::GetPerception() const {
    return perception;
}
