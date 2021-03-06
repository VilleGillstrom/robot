#pragma once

#include "RobotCommunicationMRDS.h"
#include "Perception.h"
#include "Planner.h"
#include "Motor.h"

class Navigator {
public:
    enum RobotRoutine {
        EXPLORE,
        GOAL,
        REACTING
    };

    Navigator(Cartographer &cartographer);

    Navigator(Cartographer &cartographer,
              const std::shared_ptr<RobotCommunicationMRDS> &communicator,
              const std::shared_ptr<Planner> &planner,
              const std::shared_ptr<Motor> &motor);


    bool IsNextTargetInPathWithinRange();
    void MoveTowardNextTargetInPath();
    void SelectNextTargetInPath();
    bool HasReachedGoal();
    void UpdateMotor();
    void SetSpeedLimit(double limit);

    void Navigate();

    void StartExploring();
    void ExploreRoutine();
    void StartGoalRoutine();
    void GoalRoutine();
    void ReactOverride();

    void FindNewTarget();


    std::vector<glm::ivec2> GetCurrentPath() const;
    glm::ivec2 GetCurrentTargetCell() const;

private:
    //std::shared_ptr<RobotCommunicationMRDS> communicator;
    std::shared_ptr<Planner> planner;
    std::shared_ptr<Motor> motor;
    Cartographer &cartoGrapher;


    int nextTargetIdx;
    bool hasTarget;
    std::vector<glm::ivec2> pathToTarget;

    float robotRange = 2.5; /* Is considered having reached a point if within this distance*/



    bool HasTarget();
    glm::ivec2 NextTargetCell() const;
    glm::dvec3 NextTargetLocation() const;

    bool IsCellWithinRange(glm::ivec2 cell);

    glm::dvec3 targetForwardVector;
    double targetSpeed;

    RobotRoutine routine;

    double speedLimit;
};

