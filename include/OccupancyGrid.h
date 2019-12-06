#pragma once

#include <vector>
#include <iostream>
#include <cmath>

class OccupancyGrid {
private:

    std::vector<std::vector<double>> Grid;
public:

    OccupancyGrid(int xmin, int ymin, int xmax, int ymax);


    void UpdatePosition(double x, double y, double value) {
        int row = std::lround(y -Ymin) ;
        int column = std::lround(x -Xmin) ;
        if (row < 0 || row >= Grid.size()) {
            std::cerr << "Bad row in UpdateCell()";
            return;
        }
        if (column < 0 || column >= Grid[0].size()) {
            std::cerr << "Bad column in UpdateCell()";
            return;
        }
        //std::cout << "row" << row << ", col:" << column;

        Grid[row][column] = value;
    }

    int Xmin;
    int Xmax;
    int Ymin;
    int Ymax;

    const std::vector<std::vector<double>> & GetGrid() const {
        return Grid;
    }
};


