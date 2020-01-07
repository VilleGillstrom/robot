//
// Created by twentyletters on 2020-01-06.
//

#include "include/ReactiveControl.h"

void ReactiveControl::React() {

    float minDistance = 50;
    std::vector<double> distances = perception->GetEchoDistances();
    for (int i = 135-20; i < 135+20 ; ++i) {
        for (float distance : distances) {
            minDistance = std::min(minDistance, distance);
        }
    }

    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
    auto timeSinceLastReact = std::chrono::duration_cast<std::chrono::milliseconds>(now - begin).count();
    if(minDistance < 1  && timeSinceLastReact > 5000) {
        begin = std::chrono::steady_clock::now();
        navigator->StartExploring(); //Restart exploring, recompute new path
        std::cerr << "REACT TIME" << std::endl;
        navigator->SetSpeedLimit(0.15);
    } else {
        navigator->SetSpeedLimit(2);
    }


}
