

#include "include/OccupancyGrid.h"

OccupancyGrid::OccupancyGrid(int xmin, int ymin, int xmax, int ymax)
        : Xmin(xmin), Xmax(xmax), Ymin(ymin), Ymax(ymax) {
    int width = xmax - xmin;
    int height = ymax - ymin;
    Grid = std::vector<std::vector<double>>(width, std::vector<double>(height, 0.5));
}

int OccupancyGrid::Columns() const {
    return Grid.size();
}

int OccupancyGrid::Rows() const {
    return Columns() > 0 ? Grid[0].size() : 0;
}

const std::vector<std::vector<double>> &OccupancyGrid::GetGrid() const {
    return Grid;
}


void OccupancyGrid::UpdateLocation(double x, double y, double value) {
    int row = std::lround(y - Ymin);
    int column = std::lround(x - Xmin);
    UpdateCell(row, column, value);
}


void OccupancyGrid::UpdateCell(int row, int column, double value) {
    if (!ValidRow(row)) {
        std::cerr << "Bad row in UpdateCell(): " << row << std::endl;
        return;
    }
    if (!ValidColumn(column)) {
        std::cerr << "Bad column in UpdateCell()" << column << std::endl;
        return;
    }
    Grid[row][column] = value;
}

bool OccupancyGrid::ValidRow(int row) const { return row >= 0 && row < Rows(); }

bool OccupancyGrid::ValidColumn(int column) const { return column >= 0 && column < Columns(); }
