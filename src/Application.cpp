//
// Created by twentyletters on 2020-01-06.
//

#include "include/Application.h"

void Application::SetMapVisualizer(const std::shared_ptr<MapVisualizer> &mapvisualizer) {
    this->mapvisualizer = mapvisualizer;
    if(robot) {
        mapvisualizer->SetRobot(robot);
    }
    mapvisualizer->Update();
}

void Application::SetRobot(const std::shared_ptr<Robot> &robot) {
    this->robot = robot;
}
