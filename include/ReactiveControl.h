#pragma once

#include "Cartographer.h"
#include "Perception.h"
#include <memory>
#include "Cartographer.h"
#include "Navigator.h"
#include <chrono>
#include <utility>

//Will start react if detects obstacle  closer than dangerDistance distance
static const float dangerDistance = 3;

/**
 *  Responsible for reacting
 */
class ReactiveControl {
public:
    ReactiveControl(std::shared_ptr<Navigator> navigator, std::shared_ptr<Perception> perception, std::shared_ptr<Motor> motor,
            Cartographer& cartographer_)
            : navigator(std::move(navigator)),
            perception(std::move(perception)),
            motor(std::move(motor)),
            cartographer(cartographer_)
    {
        state = OFF;
    }

    void React();

private:
    std::shared_ptr<Navigator> navigator;
    std::shared_ptr<Perception> perception;
    std::shared_ptr<Motor> motor;
    Cartographer& cartographer;


    enum ReactState {
        ROTATING,
        OFF,
    };

    ReactState state;
    glm::dvec3 targetForward;

    void ReactRoutine();
    float GetMinDistanceInFront();
    void StartReactRoutine();


    void NotifyCartographer(glm::ivec2 cell);

    glm::ivec2 GetMinDistanceCellInFront();
};


