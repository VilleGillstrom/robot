#pragma once

#include "Cartographer.h"
#include "Perception.h"
#include <memory>
#include "Cartographer.h"
#include "Navigator.h"
#include <chrono>

/**
 *  Responsible for reacting
 */
class ReactiveControl {
public:
    ReactiveControl(const std::shared_ptr<Navigator> &navigator, const std::shared_ptr<Perception> &perception)
            : navigator(navigator), perception(perception) {}

    void React();

private:
    std::shared_ptr<Navigator> navigator;
    std::shared_ptr<Perception> perception;

    // Help with reacting for a set amount of time
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
};


