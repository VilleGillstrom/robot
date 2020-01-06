#pragma once

#include "Cartographer.h"
#include "Perception.h"
#include <memory>
#include "Cartographer.h"
#include "Navigator.h"
#include <chrono>


class ReactiveControl {
public:
    ReactiveControl(const std::shared_ptr<Navigator> &navigator, const std::shared_ptr<Perception> &perception)
            : navigator(navigator), perception(perception) {}

    void React() {

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

        }


    }

private:
    std::shared_ptr<Navigator> navigator;
    std::shared_ptr<Perception> perception;

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
};


