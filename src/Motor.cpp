
#include "include/Motor.h"

Motor::Motor(const std::shared_ptr<RobotCommunicationMRDS> &communicator) : communicator(communicator) {}

void Motor::SetSpeedAndAngular(float speed, float angular) {
    this->_speed = speed;
    this->_angular = angular;
}

void Motor::SetSpeed(float speed) {
    this->_speed = speed;
}

void Motor::UpdateDrive() {
    communicator->SetSpeedAndAngular(_speed, _angular);
}

float Motor::ComputeAngularSpeed(const glm::dvec3 &robotForward, const glm::dvec3 &targetForward) {
    glm::dvec3 diffForward = glm::normalize(targetForward - robotForward);
    float forw_targ_dot = glm::dot(robotForward, targetForward);
    float AbsAngle = glm::acos((float) forw_targ_dot);

    glm::dvec3 RobotRight = glm::cross(robotForward, glm::dvec3(0, 0, 1));
    float angle = glm::dot(RobotRight, diffForward) > 0 ? -AbsAngle : AbsAngle;
    float angularspeed = (angle / 3.14) * 0.8;
    return angularspeed;
}
