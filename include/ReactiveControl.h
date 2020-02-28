#pragma once

#include "Cartographer.h"
#include "Perception.h"
#include <memory>
#include "Cartographer.h"
#include "Navigator.h"
#include <chrono>
#include <utility>

/**
 *  Responsible for reacting
 */
class ReactiveControl {
public:
    ReactiveControl(std::shared_ptr<Navigator> navigator, std::shared_ptr<Perception> perception, std::shared_ptr<Perception> motor)
            : navigator(std::move(navigator)), perception(std::move(perception)) {}

    void React();

private:
    std::shared_ptr<Navigator> navigator;
    std::shared_ptr<Perception> perception;
    std::shared_ptr<Perception> motor;

    // Help with reacting for a set amount of time
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
};


