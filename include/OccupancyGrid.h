#pragma once

#include <vector>
#include <iostream>
#include <cmath>
#include <glm/glm.hpp>

class OccupancyGrid {

public:
    OccupancyGrid(int xmin, int ymin, int xmax, int ymax);
    OccupancyGrid(int cellsize, int xmin, int ymin, int xmax, int ymax);


    void UpdateCell(int row, int column, double value);
    void UpdateCell(const glm::ivec2& cell, double value) {
        UpdateCell(cell.x, cell.y, value);
    }

    double GetCellValue(const glm::ivec2 & cell);

    void UpdateLocation(double x, double y, double value);



    int Xmin;
    int Xmax;
    int Ymin;
    int Ymax;

    const std::vector<std::vector<double>> & GetGrid() const;
    int Columns() const;
    int Rows() const;
    bool ValidRow(int row) const;
    bool ValidColumn(int column) const;

private:
    std::vector<std::vector<double>> Grid;
};


