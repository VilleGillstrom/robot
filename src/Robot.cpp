
#include "include/Robot.h"

Robot::Robot(const std::shared_ptr<RobotCommunicationMRDS> &RobotCommunicator)  :
        cartoGrapher(2, -100, -100, 100, 100),
        planner(cartoGrapher) {
    this->RobotCommunicator = RobotCommunicator;
    perception = std::make_shared<Perception>(RobotCommunicator);
    cartoGrapher.SetPreception(perception);
    planner.SetPerception(perception);
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

Planner &Robot::GetPlanner()  {
    return planner;
}
