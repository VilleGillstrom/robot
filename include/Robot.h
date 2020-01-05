#pragma once

#include <memory>
#include <future>
#include "RobotCommunicationMRDS.h"
#include "Cartographer.h"
#include "Perception.h"
#include "Planner.h"

class Robot {
public:

    explicit Robot(const std::shared_ptr<RobotCommunicationMRDS> &RobotCommunicator);
    void SetCommunicator(const std::shared_ptr<RobotCommunicationMRDS> &NewRobotCommunicator);

    /** Read properties for the robot through RobotCommunicator */


    double QuatToHeadingAngle(const glm::quat &Orientation) const;

    glm::dvec3 GetPosition() const;


    void Update() {
        perception->ReadSensors();
        cartoGrapher.Update();
    }


    Cartographer &GetCartographer();
    Planner &GetPlanner();
    std::shared_ptr<Perception> GetPerception() const;

private:

    std::shared_ptr<RobotCommunicationMRDS> RobotCommunicator;
    std::shared_ptr<Perception> perception;
    Cartographer cartoGrapher;
    Planner planner;


};


