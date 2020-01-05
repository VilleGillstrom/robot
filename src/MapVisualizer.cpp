
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

void MapVisualizer::PaintLaserView(QPixmap &pixmap) const {
    QPainter painter(&pixmap);
    QPen pen;
    pen.setStyle(Qt::PenStyle::NoPen);
    painter.setPen(pen);

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

    painter.setBrush(QBrush(QColor(0,255,0, 50)));


    painter.drawPie(rectangle, startAngle, spanAngle);
    painter.end();
}

void MapVisualizer::PaintRobot(QPixmap &pixmap) const {
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
            double p = (1 -grid[r][c]) * 255;
            int index = RowColTo1D(r, c, width);
            colormap[index] = (qRgba(p, p, p, 255));
            //int red = ((double)c /(double) width) * 255;
            //int green = ((double)r / (double)height) * 255;
            //colormap[index] = (qRgba(red, green, 0, 255));

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
