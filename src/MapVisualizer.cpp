#include <iostream>
#include <vector>
#include <unordered_set>
#include "include/MapVisualizer.h"

unsigned int MapVisualizer::GetHeight() const {
    return robot->GetCartographer().GetProbablityGrid().at(0).size();
}

unsigned int MapVisualizer::GetWidth() const {
    return robot->GetCartographer().GetProbablityGrid().size();
}

void MapVisualizer::DrawCellsInRange(QPixmap &pixmap, double range)  {
    QPainter painter(&pixmap);
    auto& cg = robot->GetCartographer();
    std::vector<glm::ivec2> cells;
    cg.GetCellsInRange(range, cells);

    for(auto &cell : cells) {
        auto ml = WorldLocationToMapLocation(cg.CellToWorldLocation(cell));
        auto rect = QRect(ml.x, ml.y, 1, 1);
        painter.fillRect(rect, QColor(0,0,255,40));
    }
    painter.end();
}

void MapVisualizer::PaintLaserView(QPixmap &pixmap)  {
    QPainter painter(&pixmap);
    painter.setBrush(QBrush(QColor(0,255,0, 50)));
    painter.setPen(QPen (Qt::black, 1, Qt::NoPen));

    glm::ivec2 rp = GetRobotPositionInMap();
    const std::shared_ptr<Perception> &Perception = robot->GetPerception();

    const glm::dvec3 &ll = Perception->GetLaserLeftEnd();
    const glm::dvec3 &lr = Perception->GetLaserRightEnd();

    auto mll = WorldLocationToMapLocation(ll);
    auto mlr = WorldLocationToMapLocation(lr);

 //   painter.drawLine(rp.x, rp.y, mll.x, mll.y);
   // painter.drawLine(rp.x, rp.y, mlr.x, mlr.y);


    auto sa = glm::degrees(-Perception->GetHeading() + Perception->GetStartAngle());
    auto span = glm::degrees(Perception->GetLaserSpan());
    int maxRange = Perception->GetLaserMaxRange() / robot->GetCartographer().CellSize();

    QRect rectangle(rp.x - maxRange, rp.y - maxRange, maxRange*2, maxRange*2);

    int startAngle = sa * 16;
    int spanAngle = span * 16;



    painter.drawPie(rectangle, startAngle, spanAngle);
    painter.end();
}

void MapVisualizer::PaintRobot(QPixmap &pixmap) {
    QPainter painter(&pixmap);
    glm::ivec2 v = GetRobotPositionInMap();
    auto rect = QRect(v.x-1, v.y-1, 2, 2);
    painter.fillRect(rect, QColor(255,0,0));
    painter.end();
}

void MapVisualizer::FillOccupancyGrid(const std::vector<std::vector<double>> &grid, int width, int height,
                                      QVector<QRgb> &colormap) const {
    for (int c = 0; c < width; c++) {
        for (int r = 0; r < height; r++) {
            double p = (1-grid[r][c]) * 255;
            int index = RowColTo1D(r, c, width);
            colormap[index] = (qRgba(p, p, p, 255));

        }
    }
}

glm::ivec2 MapVisualizer::GetRobotPositionInMap() const {
    return WorldLocationToMapLocation(robot->GetPosition());
}

glm::ivec2 MapVisualizer::WorldLocationToMapLocation(const glm::dvec3 &v) const {
    auto& cg = robot->GetCartographer();
    glm::ivec2 cell = cg.WorldLocationToCell(v);
    return {cell.y, cell.x}; //Invert since cell.x = row, cell.y = column

}

void MapVisualizer::PaintFrontier(QPixmap &pixmap) {
    QPainter painter(&pixmap);
    painter.setPen(QPen(QColor(0,0,255,255), 1, Qt::PenStyle::SolidLine));

    Planner& planner = robot->GetPlanner();
    std::vector<Frontier> frontiers;
    std::vector<glm::ivec2> pos;
    planner.FindFrontiers(frontiers);
    unsigned int numfrontiers = frontiers.size();
    int count = 0;
    std::cout << numfrontiers;
    for(const Frontier& frontier : frontiers) {
        QColor color = count < distinct_colors.size() ? distinct_colors[count] : QColor(0, 0, 255);
        painter.setPen(QPen(color, 1, Qt::PenStyle::SolidLine));

        for(const glm::ivec2& point : frontier.GetCells()) {
            painter.drawPoint(point.y, point.x);
        }

        painter.setPen(QPen(Qt::red, 1, Qt::PenStyle::SolidLine));
        glm::ivec2 centroid = frontier.GetCentroid();
        painter.drawPoint(centroid.y, centroid.x);
        count ++;

    }


}


void MapVisualizer::PaintPlannedPath(QPixmap &pixmap) {
    QPainter painter(&pixmap);

    Navigator& navigator = robot->GetNavigator();
    auto path = navigator.GetCurrentPath();
    auto nextTargetCell = navigator.GetCurrentTargetCell();


    painter.setPen(QPen(QColor(0,255,0,255), 1, Qt::PenStyle::SolidLine));
    for(auto c : path) {
        painter.drawPoint(c.y, c.x);
    }
    painter.setPen(QPen(QColor(0,128,128,255), 1, Qt::PenStyle::SolidLine));
    painter.drawPoint(nextTargetCell.y, nextTargetCell.x);
}