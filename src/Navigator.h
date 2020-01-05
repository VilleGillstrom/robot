#pragma once

class Navigator {
public:
    Navigator();

    void SetSpeed(int speed) {

    }

    std::shared_ptr<RobotCommunicatorMRDS> communicator;
};

