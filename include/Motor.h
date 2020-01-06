#pragma once

#include <memory.h>
#include "RobotCommunicationMRDS.h"


class Motor {
public:
    Motor(const std::shared_ptr<RobotCommunicationMRDS> &communicator);

    void SetSpeed(float speed){
        communicator->SetSpeed(speed);
    }

    void SetSpeedAndAngular(float speed, float angular){
        communicator->SetSpeedAndAngular(speed, angular);
    }
private:
    std::shared_ptr<RobotCommunicationMRDS> communicator;
};

