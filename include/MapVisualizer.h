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
        mw.setMinimumSize(800,900);
        mw.show();
    }

    void SetRobot(const std::shared_ptr<Robot> &robot) {
        this->Robot = robot;
        const Cartographer &cg = robot->GetCartographer();

    }

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

        pixmap = pixmap.transformed(QTransform().scale(1, -1));


        label->setPixmap(pixmap);
    }

    void PaintLaserView(QPixmap &pixmap) const {
        QPainter painter(&pixmap);

        glm::ivec2 rp = GetRobotPositionInMap();
        const std::shared_ptr<Perception> &Perception = Robot->GetPerception();

        const glm::dvec3 &ll = Perception->GetLaserLeftEnd();
        const glm::dvec3 &lr = Perception->GetLaserRightEnd();

        auto mll = WorldLocationToMapLocation(ll);
        auto mlr = WorldLocationToMapLocation(lr);
        
        painter.drawLine(rp.x, rp.y, mll.x, mll.y);
        painter.drawLine(rp.x, rp.y, mlr.x, mlr.y);


        auto sa = glm::degrees(-Perception->GetHeading() + Perception->GetStartAngle());
        auto span = glm::degrees(Perception->GetLaserSpan());
        int maxRange = Perception->GetLaserMaxRange();

        QRect rectangle(rp.x - maxRange, rp.y - maxRange, maxRange*2, maxRange*2);

        int startAngle = sa * 16;
        int spanAngle = span * 16;

        painter.setBrush(QBrush(QColor(0,255,0, 50)));
        painter.drawPie(rectangle, startAngle, spanAngle);
        painter.end();
    }

    void DrawRobot(QPixmap &pixmap) const {
        QPainter painter(&pixmap);
        glm::ivec2 v = GetRobotPositionInMap();
        auto rect = QRect(v.x-1, v.y-1, 2, 2);
        painter.fillRect(rect, QColor(255,0,0));
        painter.end();
    }

    void FillOccupancyGrid(const std::vector<std::vector<double>> &grid, int width, int height, QVector<QRgb> &colormap) const {
        for (int c = 0; c < width; c++) {
            for (int r = 0; r < height; r++) {
                double p = (1 -grid[r][c]) * 255;
                int index = RowColTo1D(r, c, width);
                colormap[index] = (qRgba(p, p, p, 255));
                //int red = ((double)c /(double) width) * 255;
                //int green = ((double)r / (double)height) * 255;
                //colormap[index] = (qRgba(red, green, 0, 255));

            }
        }
    }

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


