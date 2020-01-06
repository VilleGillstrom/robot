
#include "include/Robot.h"

Robot::Robot(const std::shared_ptr<RobotCommunicationMRDS> &robotCommunicator) :
        cartoGrapher(2, -100, -100, 100, 100) {
    this->robotCommunicator = robotCommunicator;
    perception = std::make_shared<Perception>(robotCommunicator);
    planner = std::make_shared<Planner>(cartoGrapher, perception);
    motor = std::make_shared<Motor>(robotCommunicator);
    cartoGrapher.SetPreception(perception);

    navigator = std::make_shared<Navigator>(cartoGrapher, robotCommunicator, planner, motor);
}


void Robot::SetCommunicator(const std::shared_ptr<RobotCommunicationMRDS> &NewRobotCommunicator) {
    this->robotCommunicator = NewRobotCommunicator;
}



glm::dvec3 Robot::GetPosition() const {
    return perception->GetLaserLocation();
}

Cartographer &Robot::GetCartographer() {
    return cartoGrapher;
}

std::shared_ptr<Perception> Robot::GetPerception() const {
    return perception;
}

Planner &Robot::GetPlanner() {
    return *planner;
}

Navigator &Robot::GetNavigator() {
    return *navigator;
}
