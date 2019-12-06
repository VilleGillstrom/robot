#pragma once

#include <QApplication>
#include <Qlabel>
#include <QtWidgets/QGridLayout>
#include "MainWindow.h"
#include "Robot.h"

class MapVisualizer {


public:
    MapVisualizer() {
        gridLayout = new QGridLayout();
        label = new QLabel();
        label->setScaledContents(true);
        mw.setLayout(gridLayout);
        gridLayout->addWidget(label);
        mw.show();
    }

    void SetRobot(const std::shared_ptr<Robot> &robot) {
        this->Robot = robot;
        const Cartographer &cg = robot->GetCartographer();

    }

    void Show() {
        if(!Robot) {
            return;
        }

        const Cartographer &cg = Robot->GetCartographer();
        const auto& grid = cg.GetProbablityGrid();

        int width = GetWidth();
        int height = GetHeight();

        QVector<QRgb> colormap(width * height);
        FillOccupancyGrid(grid, width, height, colormap);
        DrawRobot(colormap);


        auto image = QImage((uchar *) colormap.data(), width, height, QImage::Format_ARGB32);
        QPixmap pixmap = QPixmap::fromImage(image);
        pixmap = pixmap.transformed(QTransform().scale(1, -1));
        label->setPixmap(pixmap);
    }

    void DrawRobot(QVector<QRgb> &colormap) const {
        glm::ivec2 v = GetRobotPositionInMap();
        unsigned int width = GetWidth();
        int index = RowColTo1D(v.y, v.x, width);
        colormap[index] = qRgba(255, 0, 0, 255);
    }

    void FillOccupancyGrid(const std::vector<std::vector<double>> &grid, int width, int height, QVector<QRgb> &colormap) const {
        for (int c = 0; c < width; c++) {
            for (int r = 0; r < height; r++) {
                double p = (1 -grid[r][c]) * 255;
                int index = RowColTo1D(r, c, width);
                colormap[index] = (qRgba(p, p, p, 255));
            }
        }
    }

    glm::ivec2 GetRobotPositionInMap() const {
        glm::dvec3 v = Robot->GetPosition();
        auto& cg = Robot->GetCartographer();
        long row = std::lround(v.x) - cg.GetXMin();
        long col = std::lround(v.y) - cg.GetYMin();
        return glm::ivec2(row, col);
    }

    int RowColTo1D(int r, int c, int width) const {
        return r * width + c;
    }

private:
    std::shared_ptr<Robot> Robot;
    MainWindow mw;
    QGridLayout *gridLayout;
    QLabel *label;


    unsigned int GetHeight() const;
    unsigned int GetWidth() const;

};


