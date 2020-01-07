#pragma once


#include <memory>
#include <bits/shared_ptr.h>
#include "Robot.h"
#include "MapVisualizer.h"

class Application {


public:
    Application() = default;
    void SetRobot(const std::shared_ptr<Robot>& robot);

    void SetMapVisualizer(const std::shared_ptr<MapVisualizer>& mapvisualizer);

    void Tick() {
        robot->Tick();
        if(mapvisualizer) {
            mapvisualizer->Update();
        }
    }

private:
    std::shared_ptr<Robot> robot;
    std::shared_ptr<MapVisualizer> mapvisualizer;
};
