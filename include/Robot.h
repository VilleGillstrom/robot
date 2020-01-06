#pragma once

#include <memory>
#include <future>
#include "RobotCommunicationMRDS.h"
#include "Cartographer.h"
#include "Perception.h"
#include "Planner.h"
#include "Navigator.h"

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
        navigator->Navigate();
    }


    Cartographer &GetCartographer();
    Planner &GetPlanner();
    Navigator &GetNavigator();
    std::shared_ptr<Perception> GetPerception() const;

private:

    std::shared_ptr<RobotCommunicationMRDS> robotCommunicator;
    std::shared_ptr<Perception> perception;
    std::shared_ptr<Planner> planner;
    std::shared_ptr<Navigator> navigator;

    Cartographer cartoGrapher;


};


