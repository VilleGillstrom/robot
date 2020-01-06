#pragma once

#include "RobotCommunicationMRDS.h"
#include "Perception.h"
#include "Planner.h"
#include "Motor.h"

class Navigator {
public:
    enum RobotRoutine {
        EXPLORE,
        GOAL
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
    void UpdateRobotDrive();

    void Navigate() {
        //std::cout << "current target: (" << nextTargetIdx << "/" << pathToTarget.size() << ")" << std::endl;

        if (routine == RobotRoutine::GOAL) {

            std::cout << "Goal stuff" << std::endl;
            GoalRoutine();
        } else if (routine == RobotRoutine::EXPLORE) {
            std::cout << "Exploring" << std::endl;
            ExploreRoutine();
        }
        UpdateRobotDrive();
    }

    void StartExploring() {
        //Initialization
        pathToTarget.clear();
        nextTargetIdx = -1;
        hasTarget = false;

        routine = EXPLORE;
    }

    void ExploreRoutine() {
        if (HasReachedGoal()) {
            StartGoalRoutine();
            return;
        }
        if (!HasTarget()) {
            FindNewTarget();
        }
        SelectNextTargetInPath();
        MoveTowardNextTargetInPath();
    }

    void StartGoalRoutine() {
        //Initialization
        targetForwardVector = -cartoGrapher.RobotForwardVector();
        communicator->SetSpeedAndAngular(0, 2);
        targetSpeed = 0;
        routine = GOAL;
    }

    void GoalRoutine() {
        float d = glm::dot(targetForwardVector, cartoGrapher.RobotForwardVector());
        if ((1 - d) < 0.2) {
            //Has rotated
            StartExploring();
        }
    }


    void FindNewTarget() {
        pathToTarget = planner->ComputePath();
        hasTarget = true;
        nextTargetIdx = 0;
    }


    std::vector<glm::ivec2> GetCurrentPath() const {
        return pathToTarget;
    }


    glm::ivec2 GetCurrentTargetCell() const {
        return nextTargetIdx > 0 ? NextTargetCell() : glm::ivec2(-1, -1);
    }

private:
    std::shared_ptr<RobotCommunicationMRDS> communicator;
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

    float ComputeAngularSpeed(const glm::dvec3 &robotForward, const glm::dvec3 &targetForward) const;

    bool IsCellWithinRange(glm::ivec2 cell);

    glm::dvec3 targetForwardVector;
    float targetSpeed;

    RobotRoutine routine;

};

