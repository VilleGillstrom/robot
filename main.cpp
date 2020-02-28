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


bool VerifyURL(const std::string &url, std::shared_ptr<RobotCommunicationMRDS> &RobotCommunicator);

bool VerifyMapSize();

bool VerifyMapSize(int xmin, int xmax, int ymin, int ymax);

int main(int argc, char **argv) {
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

    QApplication app(argc, argv, false);

    //return 0;

    auto RobotCommunicator = std::make_shared<RobotCommunicationMRDS>();
    if (!RobotCommunicator->SetURL(url)) {
        std::cerr << "Bad url: " << url << ", needs to be on format: http://xyz:12345";
        exit(EXIT_FAILURE);
    }

    if (!VerifyURL(url, RobotCommunicator)) {
        exit(EXIT_FAILURE);
    }

    if (!VerifyMapSize(xmin, xmax, ymin, ymax)) {
        exit(EXIT_FAILURE);
    }


    auto robot = std::make_shared<Robot>(RobotCommunicator, xmin, ymin, xmax, ymax);
    auto mapVisualizer = std::make_shared<MapVisualizer>(showGUI == "1");


    Application robo_app;
    robo_app.SetRobot(robot);
    robo_app.SetMapVisualizer(mapVisualizer);


    auto timer = new QTimer(&app);
    QTimer::connect(timer, &QTimer::timeout, [&] {
        std::cout << "Loop" << std::endl;
        robo_app.Tick();

    });

    timer->start();


    return QApplication::exec();
}

bool VerifyMapSize(int xmin, int xmax, int ymin, int ymax) {
    if (ymin >= ymax || xmin >= xmax) {
        std::cerr << "max must be larger than min";
        return false;
    }
    return true;
}


bool VerifyURL(const std::string &url, std::shared_ptr<RobotCommunicationMRDS> &RobotCommunicator) {
    if (!RobotCommunicator->TestUrlConnection()) {
        std::cerr << "Unable to connect to " << url;
        return false;
    }
    return true;
}

