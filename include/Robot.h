#pragma once

#include <memory>
#include <future>
#include "RobotCommunicationMRDS.h"
#include "Cartographer.h"
#include "Perception.h"
#include "Planner.h"
#include "Navigator.h"
#include "Motor.h"
#include "ReactiveControl.h"

class Robot {
public:

    explicit Robot(const std::shared_ptr<RobotCommunicationMRDS> &RobotCommunicator);
    Robot(const std::shared_ptr<RobotCommunicationMRDS> &RobotCommunicator, int xmin, int ymin, int xmax, int ymax);

    /** Read properties for the robot through RobotCommunicator */
    glm::dvec3 GetPosition() const;


    void Tick() {
        perception->ReadSensors();
        cartoGrapher.Update();
        navigator->Navigate();
        reactivecontrol->React();
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
    std::shared_ptr<Motor> motor;
    std::shared_ptr<ReactiveControl> reactivecontrol;

    Cartographer cartoGrapher;


};


