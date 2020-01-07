#pragma once

#include <QApplication>
#include <QLabel>
#include <QtWidgets/QGridLayout>
#include <QtGui/QPainter>
#include "MainWindow.h"
#include "Robot.h"


/**
 * Responsible for visualize the cratgrapher in live GUI and save to image
 */
class MapVisualizer {


public:
    MapVisualizer(bool showGUI);
    void SetRobot(const std::shared_ptr<Robot> &robot);
    void Update();

private:
    std::shared_ptr<Robot> robot;
    std::shared_ptr<MainWindow> mw;
    QVBoxLayout *boxLayout;
    QLabel *label;
    QPixmap pixmap; //Pixmap representation of cartogapher

    /* Ensure an image is written to every 5 seconds */
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();


    /* Get the map width, will be equal to columns in the occupancyGrid */
    unsigned int GetHeight() const;

    /* Get the map width, will be equal to columns in the occupancyGrid */
    unsigned int GetWidth() const;

    //Some hard coded colors to paint frontiers
    std::vector<QColor> distinct_colors =
            {
                    QColor(255, 0, 0),
                    QColor(0, 255, 0),
                    QColor(0, 0, 255),
                    QColor(255, 255, 0),
                    QColor(0, 255, 255),
                    QColor(255, 0, 255),

                    QColor(255, 128, 0),
                    QColor(255, 0, 128),
                    QColor(255, 128, 128),

                    QColor(255, 128, 255),
                    QColor(255, 255, 128),

                    QColor(128, 255, 255),
                    QColor(128, 255, 128),

                    QColor(128, 128, 255),

            };

    void PaintLaserView(QPixmap &pixmap);   // Paint cells the can be seen by the laser
    void PaintRobot(QPixmap &pixmap);       // Paint the robot
    void PaintFrontier(QPixmap &pixmap);    // Paint the frontiers
    void PaintPlannedPath(QPixmap &pixmap); // Paint planned path
    void DrawCellsInRange(QPixmap &pixmap, double range); // Paint cells within range from robot

    int RowColTo1D(int r, int c, int width) const;
    glm::ivec2 GetRobotPositionInMap() const;   // Get the robots position to position in the map
    glm::ivec2 WorldLocationToMapLocation(const glm::dvec3 &v) const;    // Convert a world location to map location

    // Fill colormap with values from grid
    void FillOccupancyGrid(const std::vector<std::vector<double>> &grid, int width, int height,
                           QVector<QRgb> &colormap) const;
};


