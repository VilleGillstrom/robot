#pragma once


#include <memory>
#include <bits/shared_ptr.h>
#include "Robot.h"
#include "MapVisualizer.h"

/**
 * Application containing robot and visualizer to visualize the robot and its map
 */
class Application {
public:
    Application() = default;
    void SetRobot(const std::shared_ptr<Robot>& robot); // Set the robot to run in the application
    void SetMapVisualizer(const std::shared_ptr<MapVisualizer>& mapvisualizer); // Set the visualizer
    void Tick(); // Called to update robot and visualizer

private:
    std::shared_ptr<Robot> robot;
    std::shared_ptr<MapVisualizer> mapvisualizer;
};
