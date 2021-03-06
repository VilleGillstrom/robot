
#include "include/Navigator.h"

#include <utility>

Navigator::Navigator(Cartographer &cartographer) : cartoGrapher(cartographer) {

}

Navigator::Navigator(Cartographer &cartographer, const std::shared_ptr<RobotCommunicationMRDS> &communicator,
                     const std::shared_ptr<Planner> &planner,
                     const std::shared_ptr<Motor> &motor)
        : cartoGrapher(cartographer),
          planner(planner),
          motor(motor),
          nextTargetIdx(-1),
          hasTarget(false),
          speedLimit(2),
          routine(RobotRoutine::EXPLORE) {
}





bool Navigator::HasTarget() {
    return hasTarget;
}

bool Navigator::IsNextTargetInPathWithinRange() {
    return nextTargetIdx < 0 ? true : IsCellWithinRange(NextTargetCell());
}

bool Navigator::IsCellWithinRange(glm::ivec2 cell) {
    auto location = cartoGrapher.CellToWorldLocation(cell);
    auto currentLocation = cartoGrapher.RobotLocation();
    auto distance = glm::distance(location, currentLocation);
    return distance < robotRange;
}

glm::dvec3 Navigator::NextTargetLocation() const {
    //If no valid next target cell, return robot location at next target location
    return nextTargetIdx < 0 ? cartoGrapher.RobotLocation() : cartoGrapher.CellToWorldLocation(NextTargetCell());
}


void Navigator::MoveTowardNextTargetInPath() {
    glm::dvec3 nextLocation = NextTargetLocation();
    glm::dvec3 CurrentLocation = cartoGrapher.RobotLocation();

    if (glm::length(nextLocation - CurrentLocation) < 0.01) {
        //If the target location is the current location dont move, next iteration if Navigate() will take us away from this
        targetForwardVector = glm::vec3(1, 0, 0);
        targetSpeed = 0;
    } else {
        targetForwardVector = glm::normalize(nextLocation - CurrentLocation);
        //Rotate on spot if large angle toward target forward
        bool IsFacingTowardsTarget = glm::dot(cartoGrapher.RobotForwardVector(), targetForwardVector) > 0.8;
        targetSpeed = IsFacingTowardsTarget ? 0.9 : 0;
    }


}


void Navigator::SelectNextTargetInPath() {
    for (int i = nextTargetIdx; i < pathToTarget.size(); i++) {
        nextTargetIdx = i;
        if (!IsNextTargetInPathWithinRange()) {
            break;
        }
    }
}

bool Navigator::HasReachedGoal() {
    return (nextTargetIdx == pathToTarget.size() - 1) && pathToTarget.size() > 0 &&
           IsCellWithinRange(pathToTarget.back());
}

glm::ivec2 Navigator::NextTargetCell() const {
    return pathToTarget[nextTargetIdx];
}

void Navigator::UpdateMotor() {
    glm::dvec3 robotForward = cartoGrapher.RobotForwardVector();
    float angularspeed = Motor::ComputeAngularSpeed(robotForward, targetForwardVector);
    //std::cout << "speed: " << targetSpeed << "angularspeed: " << angularspeed << std::endl;
    float limitedSpeed = std::min(targetSpeed, speedLimit);
    motor->SetSpeedAndAngular(limitedSpeed, angularspeed);
}

void Navigator::Navigate() {
    //std::cout << "current target: (" << nextTargetIdx << "/" << pathToTarget.size() << ")" << std::endl;

    switch(routine) {
        case EXPLORE:
            std::cout << "Exploring" << std::endl;
            ExploreRoutine();
            UpdateMotor();
            break;
        case GOAL:
            std::cout << "Goal stuff" << std::endl;
            GoalRoutine();
            UpdateMotor();
            break;
        case REACTING:
            // Handled by ReactiveControl
            break;
    }
}

void Navigator::StartExploring() {
    //Initialization
    pathToTarget.clear();
    nextTargetIdx = -1;
    hasTarget = false;
    routine = EXPLORE;
}

void Navigator::ExploreRoutine() {
    if (HasReachedGoal()) {
        StartGoalRoutine();
        return;
    }
    if (!HasTarget()) {
        FindNewTarget();
    }

    //Verify that a target was successfly found
    if (HasTarget()) {
        SelectNextTargetInPath();
        MoveTowardNextTargetInPath();
    }

}

void Navigator::StartGoalRoutine() {
    //Initialization
    targetForwardVector = -cartoGrapher.RobotForwardVector();
    motor->SetSpeedAndAngular(0, 2);
    targetSpeed = 0;
    routine = GOAL;
}

void Navigator::GoalRoutine() {
    float d = glm::dot(targetForwardVector, cartoGrapher.RobotForwardVector());
    if ((1 - d) < 0.2) {
        //Has rotated
        StartExploring();
    }
}


void Navigator::ReactOverride() {
    routine = REACTING;
}



void Navigator::FindNewTarget() {
    pathToTarget = planner->SelectPath();
    if(pathToTarget.size() > 0) {
        hasTarget = true;
        nextTargetIdx = 0;
    } else {
        hasTarget = false;
        nextTargetIdx = -1;
    }

}

std::vector<glm::ivec2> Navigator::GetCurrentPath() const {
    return pathToTarget;
}

glm::ivec2 Navigator::GetCurrentTargetCell() const {
    return nextTargetIdx > 0 ? NextTargetCell() : glm::ivec2(-1, -1);
}

void Navigator::SetSpeedLimit(double limit) {
    this->speedLimit = limit;
}


