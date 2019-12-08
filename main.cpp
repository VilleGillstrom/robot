#include <iostream>
#include "include/RobotCommunicationMRDS.h"
#include "include/MainWindow.h"

#include <curl/curl.h>
#include <QApplication>
#include <Qlabel>
#include <QtWidgets/QGridLayout>
#include <glm/gtx/string_cast.hpp>
#include <include/Robot.h>
#include <include/MapVisualizer.h>
#include <QTimer>
#include <unistd.h>
#define GLM_ENABLE_EXPERIMENTAL // enable glm::to_string

#define WIDTH 600
#define HEIGHT 600
#define STRIDE (WIDTH * 4)

unsigned char image[STRIDE*HEIGHT];

int main(int argc, char** argv) {
    auto curl = curl_easy_init();
    std::string host = "http://localhost";
    int port = 50002;

    QApplication app (argc, argv);


    auto RobotCommunicator = std::make_shared<RobotCommunicationMRDS>(host, port);
    auto robot = std::make_shared<Robot>(RobotCommunicator);


    MapVisualizer mapVisualizer;
    mapVisualizer.SetRobot(robot);
    mapVisualizer.Update();

     auto timer = new QTimer(&app);
    QTimer::connect(timer, &QTimer::timeout, [&]{
        std::cout << "Loop" << std::endl;
        robot->Update();
        mapVisualizer.Update();
       // usleep(100);
    });
    timer->start();

    return QApplication::exec();
}
