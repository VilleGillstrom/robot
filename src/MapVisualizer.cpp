#include <iostream>
#include <vector>
#include <unordered_set>
#include "include/MapVisualizer.h"

unsigned int MapVisualizer::GetHeight() const {
    return robot->GetCartographer().MapHeight();
}

unsigned int MapVisualizer::GetWidth() const {
    return robot->GetCartographer().MapWidth();

}

void MapVisualizer::DrawCellsInRange(QPixmap &pixmap, double range) {
    QPainter painter(&pixmap);
    auto &cg = robot->GetCartographer();
    std::vector<glm::ivec2> cells;
    cg.GetCellsInRobotRange(range, cells);

    for (auto &cell : cells) {
        auto ml = WorldLocationToMapLocation(cg.CellToWorldLocation(cell));
        auto rect = QRect(ml.x, ml.y, 1, 1);
        painter.fillRect(rect, QColor(0, 0, 255, 40));
    }
    painter.end();
}

void MapVisualizer::PaintLaserView(QPixmap &pixmap) {
    QPainter painter(&pixmap);
    painter.setBrush(QBrush(QColor(0, 255, 0, 50)));
    painter.setPen(QPen(Qt::black, 1, Qt::NoPen));

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
    int maxRange = 40 / robot->GetCartographer().CellSize();

    QRect rectangle(rp.x - maxRange, rp.y - maxRange, maxRange * 2, maxRange * 2);

    int startAngle = sa * 16;
    int spanAngle = span * 16;


    painter.drawPie(rectangle, startAngle, spanAngle);
    painter.end();
}

void MapVisualizer::PaintRobot(QPixmap &pixmap) {
    QPainter painter(&pixmap);
    glm::ivec2 v = GetRobotPositionInMap();
    painter.setPen(QPen(QColor(0, 0, 255, 255), 1, Qt::PenStyle::SolidLine));

    auto rect = QRect(v.x - 1, v.y - 1, 2, 2);
    painter.fillRect(rect, QColor(255, 0, 0));
    painter.end();
}

void MapVisualizer::FillOccupancyGrid(const std::vector<std::vector<double>> &grid, int width, int height,
                                      QVector<QRgb> &colormap) const {
    for (int c = 0; c < width; c++) {
        for (int r = 0; r < height; r++) {
            double p = (1 - grid[r][c]) * 255;
            int index = RowColTo1D(r, c, width);
            colormap[index] = (qRgba(p, p, p, 255));
        }
    }
}

glm::ivec2 MapVisualizer::GetRobotPositionInMap() const {
    const glm::dvec3 WorldPosition = robot->GetPosition();
    const glm::ivec2 MapPosition = WorldLocationToMapLocation(WorldPosition);
    return MapPosition;
}

glm::ivec2 MapVisualizer::WorldLocationToMapLocation(const glm::dvec3 &v) const {
    auto &cg = robot->GetCartographer();
    glm::ivec2 cell = cg.WorldLocationToCell(v);
    return {cell.y, cell.x}; //Invert since cell.x = row, cell.y = column

}

void MapVisualizer::PaintFrontier(QPixmap &pixmap) {
    QPainter painter(&pixmap);
    painter.setPen(QPen(QColor(0, 0, 255, 255), 1, Qt::PenStyle::SolidLine));

    Planner &planner = robot->GetPlanner();
    std::vector<glm::ivec2> pos;
    std::vector<Frontier> frontiers;
    planner.FindFrontiers(frontiers);
    unsigned int numfrontiers = frontiers.size();
    int count = 0;
    //  std::cout << numfrontiers;
    for (const Frontier &frontier : frontiers) {
        QColor color = count < distinct_colors.size() ? distinct_colors[count] : QColor(0, 0, 255);
        painter.setPen(QPen(color, 1, Qt::PenStyle::SolidLine));

        for (const glm::ivec2 &point : frontier.GetCells()) {
            painter.drawPoint(point.y, point.x);
        }

        painter.setPen(QPen(Qt::red, 1, Qt::PenStyle::SolidLine));
        glm::ivec2 centroid = frontier.GetCentroid();
        painter.drawPoint(centroid.y, centroid.x);
        count++;

    }


}


void MapVisualizer::PaintPlannedPath(QPixmap &pixmap) {
    QPainter painter(&pixmap);

    Navigator &navigator = robot->GetNavigator();
    auto path = navigator.GetCurrentPath();
    auto nextTargetCell = navigator.GetCurrentTargetCell();


    painter.setPen(QPen(QColor(0, 255, 255, 128), 1, Qt::PenStyle::SolidLine));
    for (auto c : path) {
        painter.drawPoint(c.y, c.x);
    }
    painter.setPen(QPen(QColor(0, 255, 128, 255), 1, Qt::PenStyle::SolidLine));
    painter.drawPoint(nextTargetCell.y, nextTargetCell.x);
}

void MapVisualizer::Update() {
    if (!robot) {
        return;
    }

    Cartographer &cg = robot->GetCartographer();
    const auto &grid = cg.GetProbablityGrid();

    int width = GetWidth();
    int height = GetHeight();

    QVector<QRgb> colormap(width * height);
    FillOccupancyGrid(grid, width, height, colormap);

    QImage image = QImage((uchar *) colormap.data(), width, height, QImage::Format_ARGB32);
    pixmap = QPixmap::fromImage(image);

    PaintLaserView(pixmap);
    PaintPlannedPath(pixmap);
    PaintRobot(pixmap);
    //  DrawCellsInRange(pixmap, 40);

    PaintFrontier(pixmap);

    pixmap = pixmap.transformed(QTransform().scale(1, -1));
    pixmap = pixmap.scaled(800, 800, Qt::IgnoreAspectRatio, Qt::FastTransformation);

    //Tick gui if applicable
    if (label)
        label->setPixmap(pixmap);

    //Save an image every 5 seconds
    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
    auto timeSinceLastImage = std::chrono::duration_cast<std::chrono::milliseconds>(now - begin).count();

    if (timeSinceLastImage > 5000) {
        //image.save("map.png",0, 100 );
        QFile file("map.png");
        file.open(QIODevice::WriteOnly);
        pixmap.save(&file, "PNG");
        file.close();
    }
}

MapVisualizer::MapVisualizer(bool showGUI) : label(nullptr) {

    if (showGUI) {
        boxLayout = new QVBoxLayout();
        label = new QLabel();
        label->setScaledContents(true);

        mw = std::make_shared<MainWindow>();
        mw->setLayout(boxLayout);
        boxLayout->addWidget(label);
        mw->setMinimumSize(800, 800);
        mw->show();
    }
}

void MapVisualizer::SetRobot(const std::shared_ptr<Robot> &robot) {
    this->robot = robot;
    const Cartographer &cg = robot->GetCartographer();

}

int MapVisualizer::RowColTo1D(int r, int c, int width) const {
    return r * width + c;
}
