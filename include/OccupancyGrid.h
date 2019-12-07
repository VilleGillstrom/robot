#pragma once

#include <vector>
#include <iostream>
#include <cmath>

class OccupancyGrid {
private:

    std::vector<std::vector<double>> Grid;
public:

    OccupancyGrid(int xmin, int ymin, int xmax, int ymax);

    void UpdateCell(int row, int column, double value);
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
};


