#pragma once

#include <QApplication>
#include <QLabel>
#include <QtWidgets/QGridLayout>
#include <QtGui/QPainter>
#include "MainWindow.h"
#include "Robot.h"

class MapVisualizer {


public:
    MapVisualizer(bool showGUI);

    void SetRobot(const std::shared_ptr<Robot> &robot);
    void DrawCellsInRange(QPixmap &pixmap, double i);
    void Update();
    void FillOccupancyGrid(const std::vector<std::vector<double>> &grid, int width, int height,
                           QVector<QRgb> &colormap) const;

    glm::ivec2 GetRobotPositionInMap() const;

    glm::ivec2 WorldLocationToMapLocation(const glm::dvec3 &v) const;

    int RowColTo1D(int r, int c, int width) const {
        return r * width + c;
    }

private:
    std::shared_ptr<Robot> robot;
    std::shared_ptr<MainWindow> mw;
    QVBoxLayout *boxLayout;
    QLabel *label;
    QPixmap pixmap; //Pixmap representation of cartogapher

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();


    /** Get the map width, will be equal to columns in the occupancyGrid */
    unsigned int GetHeight() const;

    /** Get the map width, will be equal to columns in the occupancyGrid */
    unsigned int GetWidth() const;

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

    void PaintLaserView(QPixmap &pixmap);
    void PaintRobot(QPixmap &pixmap);
    void PaintFrontier(QPixmap &pixmap);
    void PaintPlannedPath(QPixmap &pixmap);
};


