#pragma once

#include "RobotCommunicationMRDS.h"
#include "Perception.h"
#include "Planner.h"

class Navigator {
public:
    enum RobotRoutine {
        EXPLORE,
        GOAL
    };

    Navigator(Cartographer &cartographer);

    Navigator(Cartographer &cartographer, const std::shared_ptr<RobotCommunicationMRDS> &communicator,
              const std::shared_ptr<Perception> &percept, const std::shared_ptr<Planner> &planner);


    bool IsNextTargetInPathWithinRange();
    void MoveTowardNextTargetInPath();
    void SelectNextTargetInPath();
    bool HasReachedGoal();
    void UpdateRobotDrive();

    void Navigate() {
        std::cout << "current target: (" << nextTargetIdx << "/" <<  pathToTarget.size() << ")" << std::endl;

        if(routine  == RobotRoutine::GOAL) {
            GoalRoutine();
        }else if(routine  == RobotRoutine::EXPLORE) {
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
        if(HasReachedGoal()) {
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
        targetForwardVector = -perception->GetRobotForwardVector();
        communicator->SetSpeedAndAngular(0, 2);
        targetSpeed = 0;
        initializedGoalRoutine = true;
        routine = GOAL;
    }

    void GoalRoutine()  {
        float d = glm::dot(targetForwardVector, perception->GetRobotForwardVector());
        std::cerr << d;

        if((1 - d) < 0.2) {
            //Finished goal routine
            initializedGoalRoutine = false;
            StartExploring();

        }
    }



    void FindNewTarget() {
        pathToTarget = planner->ComputePath();
        hasTarget = true;
        nextTargetIdx = 0;
    }

    void SetSpeed(int speed);




    void SetCommunicator(std::shared_ptr<RobotCommunicationMRDS> new_communicator);

    void SetPerception(std::shared_ptr<Perception> new_percept);

    std::vector<glm::ivec2> GetCurrentPath() const {
        return pathToTarget;
    }



    glm::ivec2 GetCurrentTargetCell () const {
        return nextTargetIdx > 0 ? NextTargetCell() : glm::ivec2(-1,-1);
    }
private:
    std::shared_ptr<RobotCommunicationMRDS> communicator;
    std::shared_ptr<Perception> perception;
    std::shared_ptr<Planner> planner;
    Cartographer &cartoGrapher;


    int nextTargetIdx;
    bool hasTarget;
    std::vector<glm::ivec2> pathToTarget;

    float robotSize = 2; /* Size of the robot */
    float robotRange = 4; /* Is considered having reacher a point if within this distance*/


    bool initializedGoalRoutine;
    bool HasTarget();

    glm::ivec2 NextTargetCell() const;
    glm::dvec3 NextTargetLocation() const;

    float ComputeAngularSpeed(const glm::dvec3 &robotForward, const glm::dvec3 &targetForward) const;

    bool IsCellWithinRange(glm::ivec2 cell);

    glm::dvec3 targetForwardVector;
    float targetSpeed;

    RobotRoutine routine;

    void AvoidObstacles();
};

