#include "include/ReactiveControl.h"

void ReactiveControl::React() {
    if(state != OFF) {
        ReactRoutine();
    } else if(GetMinDistanceInFront() < dangerDistance){
        StartReactRoutine();
    }
}

float  ReactiveControl::GetMinDistanceInFront() {
    double minDistance = 50;
    std::vector<double> distances = perception->GetEchoDistances();
    for (int i = 135-20; i < 135+20 ; ++i) {
        minDistance = std::min(minDistance, distances[i]);
    }
    return (float) minDistance;
}

glm::ivec2  ReactiveControl::GetMinDistanceCellInFront() {
    double minDistance = 50;
    int minIdx = -1;
    std::vector<double> distances = perception->GetEchoDistances();
    for (int i = 135-20; i < 135+20 ; ++i) {
        minDistance = std::min(minDistance, distances[i]);
        minIdx = i;
    }
    glm::dvec3 v = perception->GetLaserWorldVector(minIdx);
    glm::dvec3 worldLocation = perception->GetLaserLocation() +  v * minDistance;
    glm::ivec2 dangerCell = cartographer.WorldLocationToCell(worldLocation);
    return dangerCell;
}


void ReactiveControl::StartReactRoutine() {
    navigator->ReactOverride(); // Will pause the exploring routine
    state = ROTATING;
    glm::dvec3 robotForward = perception->GetRobotForwardVector();

    //Update cartographer about dangerous cell
    glm::ivec2 dangerCell =  GetMinDistanceCellInFront();
    NotifyCartographer(dangerCell);

    targetForward = glm::rotateZ(robotForward, 20.0);
}


void ReactiveControl::NotifyCartographer(glm::ivec2 cell) {
    Grid &dangerGrid = cartographer.GetDangerGrid();
    dangerGrid.UpdateCell(cell, 1);

    // make adjacent cells also dangerous
    std::vector<glm::ivec2> adjs = cartographer.GetAdjacent(cell);
    for(glm::ivec2 adj : adjs) {
        dangerGrid.UpdateCell(adj, std::max(0.5, dangerGrid.GetCellValue(adj)));
    }
}


void ReactiveControl::ReactRoutine() {
    glm::dvec3 robotForward = perception->GetRobotForwardVector();

    if(state == ROTATING) {
        //If facing close to targetForward
        if (glm::dot(robotForward, targetForward) > 0.95) {
            float minDistance = GetMinDistanceInFront();
            if (minDistance > dangerDistance) {
                //Resume exploring
                navigator->StartExploring();
                state = OFF;
                return;
            } else {
                // Rotate some more
                glm::ivec2 dangerCell =  GetMinDistanceCellInFront();
                NotifyCartographer(dangerCell);
                targetForward = glm::rotateZ(robotForward, 20.0);
            }
        }

        float angular = Motor::ComputeAngularSpeed( robotForward, targetForward);
        motor->SetSpeedAndAngular(0.0, angular);
    }

}


