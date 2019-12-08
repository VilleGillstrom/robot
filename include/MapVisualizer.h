#pragma once

#include <QApplication>
#include <Qlabel>
#include <QtWidgets/QGridLayout>
#include <QtGui/QPainter>
#include "MainWindow.h"
#include "Robot.h"

class MapVisualizer {


public:
    MapVisualizer() {
        boxLayout = new QVBoxLayout();
        label = new QLabel();
        label->setScaledContents(true);
        mw.setLayout(boxLayout);
        boxLayout->addWidget(label);
        mw.setMinimumSize(800,800);
        mw.show();
    }

    void SetRobot(const std::shared_ptr<Robot> &robot) {
        this->Robot = robot;
        const Cartographer &cg = robot->GetCartographer();

    }

    void DrawCellsInRange(QPixmap &pixmap, double i);

    void Update() {
        if(!Robot) {
            return;
        }

        const Cartographer &cg = Robot->GetCartographer();
        const auto& grid = cg.GetProbablityGrid();

        int width = GetWidth();
        int height = GetHeight();

        QVector<QRgb> colormap(width * height);
        FillOccupancyGrid(grid, width, height, colormap);

        QImage image = QImage((uchar *) colormap.data(), width, height, QImage::Format_ARGB32);
        QPixmap pixmap = QPixmap::fromImage(image);

        PaintLaserView(pixmap);
        DrawRobot(pixmap);
      //  DrawCellsInRange(pixmap, 40);

        pixmap = pixmap.transformed(QTransform().scale(1, -1));


        label->setPixmap(pixmap);
    }

    void PaintLaserView(QPixmap &pixmap) const;
    void DrawRobot(QPixmap &pixmap) const;
    void FillOccupancyGrid(const std::vector<std::vector<double>> &grid, int width, int height, QVector<QRgb> &colormap) const;

    glm::ivec2 GetRobotPositionInMap() const {
        return WorldLocationToMapLocation(Robot->GetPosition());
    }

    glm::ivec2 WorldLocationToMapLocation(const glm::dvec3 &v) const {
        auto& cg = Robot->GetCartographer();
        int row= std::lround(v.x) - cg.GetXMin();
        int col= std::lround(v.y) - cg.GetYMin();
        return {row, col};
    }

    int RowColTo1D(int r, int c, int width) const {
        return r * width + c;
    }

private:
    std::shared_ptr<Robot> Robot;
    MainWindow mw;
    QVBoxLayout* boxLayout;
    QLabel *label;


    /** Get the map width, will be equal to columns in the occupancyGrid */
    unsigned int GetHeight() const;

    /** Get the map width, will be equal to columns in the occupancyGrid */
    unsigned int GetWidth() const;

};


