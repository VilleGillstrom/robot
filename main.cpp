#include <iostream>
#include "include/RobotCommunicationMRDS.h"

#include <curl/curl.h>
#include <QApplication>
#include <include/Robot.h>
#include <include/MapVisualizer.h>
#include <QTimer>
#include <string>
#include <include/Application.h>

#define GLM_ENABLE_EXPERIMENTAL // enable glm::to_string

#define WIDTH 600
#define HEIGHT 600
#define STRIDE (WIDTH * 4)

unsigned char image[STRIDE*HEIGHT];


int main(int argc, char** argv) {
    auto curl = curl_easy_init();

    if(argc != 7) {
        std::cerr << "Wrong number of arguments: example usage ./application  <url> <x_min> <y_min> <x_max> <y_max> <showGUI>" << std::endl;
        exit(EXIT_FAILURE);
    }

    //Get host and port
    std::string url = argv[1];
    std::string xminstr = argv[2];
    std::string yminstr = argv[3];
    std::string xmaxstr = argv[4];
    std::string ymaxstr = argv[5];
    std::string showGUI = argv[6];

    int xmin, ymin, xmax, ymax;
    std::stringstream(xminstr) >> xmin;
    std::stringstream(yminstr) >> ymin;
    std::stringstream(xmaxstr) >> xmax;
    std::stringstream(ymaxstr) >> ymax;

    QApplication app (argc, argv);


    auto RobotCommunicator = std::make_shared<RobotCommunicationMRDS>();
    RobotCommunicator->SetURL(url);



    auto robot = std::make_shared<Robot>(RobotCommunicator, xmin, ymin, xmax,ymax );
    //auto robot = std::make_shared<Robot>(RobotCommunicator, -40, -40, 40,40 );

    Application robo_app;
    robo_app.SetRobot(robot);
    if(showGUI == "1") {
        auto mapVisualizer = std::make_shared<MapVisualizer>();
        robo_app.SetMapVisualizer(mapVisualizer);
    }





    auto timer = new QTimer(&app);
    QTimer::connect(timer, &QTimer::timeout, [&]{
        std::cout << "Loop" << std::endl;
        robo_app.Tick();

    });

    timer->start();

    return QApplication::exec();
}

