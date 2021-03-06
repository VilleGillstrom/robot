
#include "include/Robot.h"

Robot::Robot(const std::shared_ptr<RobotCommunicationMRDS> &robotCommunicator) :
        Robot(robotCommunicator, -60, -60, 60, 60) {
}

Robot::Robot(const std::shared_ptr<RobotCommunicationMRDS> &robotCommunicator, int xmin, int ymin, int xmax, int ymax) :
        cartoGrapher(1.5, xmin, ymin, xmax, ymax) {
    this->robotCommunicator = robotCommunicator;
    perception = std::make_shared<Perception>(robotCommunicator);
    planner = std::make_shared<Planner>(cartoGrapher);
    motor = std::make_shared<Motor>(robotCommunicator);
    navigator = std::make_shared<Navigator>(cartoGrapher, robotCommunicator, planner, motor);
    reactivecontrol = std::make_shared<ReactiveControl>(navigator, perception, motor, cartoGrapher);
    cartoGrapher.SetPreception(perception);

}

void Robot::Tick() {
    perception->ReadSensors();
    cartoGrapher.Update();
    navigator->Navigate();
    reactivecontrol->React();
    motor->UpdateDrive();
}


glm::dvec3 Robot::GetPosition() const {
    glm::dvec3 LaserWorldLocation = perception->GetLaserLocation();
    return LaserWorldLocation;
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

