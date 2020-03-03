#pragma once

#include <memory.h>
#include "RobotCommunicationMRDS.h"

/**
 * Control the motor of the robot
 */
class Motor {
public:
    Motor(const std::shared_ptr<RobotCommunicationMRDS> &communicator);

    /* Set the speed used for next time UpdateDrive is called */
    void SetSpeed(float speed);

    /* Set the speed and angular used for next time UpdateDrive is called */
    void SetSpeedAndAngular(float speed, float angular);

    /* Update the actual drives on the robot */
    void UpdateDrive();

    //Helper function to help with decision of angular speed given a forward and target vector
    static float ComputeAngularSpeed(const glm::dvec3 &robotForward, const glm::dvec3 &targetForward);

private:
    std::shared_ptr<RobotCommunicationMRDS> communicator;

    float _speed;
    float _angular;

};

