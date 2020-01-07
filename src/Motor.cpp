//
// Created by twentyletters on 2020-01-06.
//

#include "include/Motor.h"

Motor::Motor(const std::shared_ptr<RobotCommunicationMRDS> &communicator) : communicator(communicator) {}

void Motor::SetSpeedAndAngular(float speed, float angular) {
    communicator->SetSpeedAndAngular(speed, angular);
}

void Motor::SetSpeed(float speed) {
    communicator->SetSpeed(speed);
}
