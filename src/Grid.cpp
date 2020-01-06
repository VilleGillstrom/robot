

#include "include/Grid.h"


Grid::Grid(double cellsize, int xmin, int ymin, int xmax, int ymax) : Grid(cellsize, xmin, ymin, xmax, ymax, 0.5)
{
}

Grid::Grid(double cellsize, int xmin, int ymin, int xmax, int ymax, float value)
        :Xmin(xmin), Xmax(xmax), Ymin(ymin), Ymax(ymax)  {
    int width = ceil(((xmax - xmin) / (double) cellsize));
    int height = ceil(((ymax - ymin) / (double) cellsize));


    grid = std::vector<std::vector<double>>(width, std::vector<double>(height, value));
}

int Grid::NumColumns() const {
    return NumRows() > 0 ? grid[0].size() : 0;

}

int Grid::NumRows() const {
    return grid.size();

}

const std::vector<std::vector<double>> &Grid::GetGrid() const {
    return grid;
}


void Grid::UpdateLocation(double x, double y, double value) {
    int row = std::lround(y - Ymin);
    int column = std::lround(x - Xmin);
    UpdateCell(row, column, value);
}


void Grid::UpdateCell(int row, int column, double value) {
    if (!IsValidRow(row)) {
        std::cerr << "Bad row in UpdateCell(): " << row << std::endl;
        return;
    }
    if (!IsValidColumn(column)) {
        std::cerr << "Bad column in UpdateCell()" << column << std::endl;
        return;
    }
    grid[row][column] = value;
}

bool Grid::IsValidRow(int row) const { return row >= 0 && row < NumRows(); }

bool Grid::IsValidColumn(int column) const { return column >= 0 && column < NumColumns(); }

bool Grid::IsValidCell(const glm::ivec2 &cell) const {
    return IsValidRow(cell.x) && IsValidColumn(cell.y);
}

double Grid::GetCellValue(const glm::ivec2 &cell) const {
    return  grid[cell.x][cell.y];
}

void Grid::UpdateCell(const glm::ivec2 &cell, double value) {
    UpdateCell(cell.x, cell.y, value);
}

