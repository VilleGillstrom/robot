
#include "include/Navigator.h"

#include <utility>

Navigator::Navigator(Cartographer &cartographer) : cartoGrapher(cartographer) {

}

Navigator::Navigator(Cartographer &cartographer, const std::shared_ptr<RobotCommunicationMRDS> &communicator,
                     const std::shared_ptr<Perception> &percept, const std::shared_ptr<Planner> &planner)
        : cartoGrapher(cartographer),
          communicator(communicator),
          perception(percept),
          planner(planner),
          nextTargetIdx(-1),
          hasTarget(false),
          routine(RobotRoutine::EXPLORE){
}

void Navigator::SetCommunicator(std::shared_ptr<RobotCommunicationMRDS> new_communicator) {
    communicator = std::move(new_communicator);
}

void Navigator::SetPerception(std::shared_ptr<Perception> new_percept) {
    perception = std::move(new_percept);
}

void Navigator::SetSpeed(int speed) {
    communicator->SetSpeed(speed);
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

    if(glm::length(nextLocation - CurrentLocation) < 0.01) {
        //If the target location is the current location dont move, next iteration if Navigate() will take us away from this
        targetForwardVector = glm::vec3(1,0,0);
        targetSpeed = 0;
    } else {
        targetForwardVector = glm::normalize(nextLocation - CurrentLocation);
        //Rotate on spot if large angle toward target forward
        targetSpeed =  glm::dot(perception->GetRobotForwardVector(), targetForwardVector) > 0.8 ? 0.9 : 0;

        AvoidObstacles();

    }


}

float Navigator::ComputeAngularSpeed(const glm::dvec3 &robotForward, const glm::dvec3 &targetForward) const {
    glm::dvec3 diffForward = glm::normalize(targetForward - robotForward);
    float forw_targ_dot = glm::dot(robotForward, targetForward);
    float AbsAngle = glm::acos((float) forw_targ_dot);

    glm::dvec3 RobotRight = glm::cross(robotForward, glm::dvec3(0, 0, 1));
    float angle = glm::dot(RobotRight, diffForward) > 0 ? -AbsAngle : AbsAngle;
    float angularspeed = (angle / 3.14) * 0.8;
    return angularspeed;
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

void Navigator::UpdateRobotDrive() {
    glm::dvec3 robotForward = perception->GetRobotForwardVector();
    float angularspeed = ComputeAngularSpeed(robotForward, targetForwardVector);
    std::cout << "speed: " << targetSpeed << "angularspeed: " << angularspeed << std::endl;
    communicator->SetSpeedAndAngular(targetSpeed, angularspeed);
}

void Navigator::AvoidObstacles() {
    std::vector<double> echoes = perception->GetEchoDistances();

//    for(int i = (271/2)-50 ; i < (271/2)+50; i++) {
//        glm::dvec3 laserVector = perception->GetLaserWorldVector(i);
//        float power = 1/echoes[i]; /* Close = bigger poer *&
//        if()
//    }


}


