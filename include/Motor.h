#pragma once

#include <memory.h>
#include "RobotCommunicationMRDS.h"

/**
 * Control the motor of the robot
 */
class Motor {
public:
    Motor(const std::shared_ptr<RobotCommunicationMRDS> &communicator);

    void SetSpeed(float speed);
    void SetSpeedAndAngular(float speed, float angular);
private:
    std::shared_ptr<RobotCommunicationMRDS> communicator;
};

