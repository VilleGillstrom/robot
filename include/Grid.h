#pragma once

#include <vector>
#include <iostream>
#include <cmath>
#include <glm/glm.hpp>
/**
 * A 2D grid, primarily used of occopancy  gui
 */
class Grid {

public:

    Grid(double cellsize, int xmin, int ymin, int xmax, int ymax);
    Grid(double cellsize, int xmin, int ymin, int xmax, int ymax, float value);

    /** Set cell value */
    void UpdateCell(int row, int column, double value);
    /** Set cell value */
    void UpdateCell(const glm::ivec2& cell, double value);
    /** Find cell at x and y location, and set value*/
    void UpdateLocation(double x, double y, double value);

    /** Get cell value */
    double GetCellValue(const glm::ivec2 & cell) const;

    int Xmin;
    int Xmax;
    int Ymin;
    int Ymax;

    const std::vector<std::vector<double>> & GetMatrix() const;
    int NumColumns() const;
    int NumRows() const;
    bool IsValidRow(int row) const;
    bool IsValidColumn(int column) const;
    bool IsValidCell(const glm::ivec2& cell) const;
private:
    std::vector<std::vector<double>> grid;
};


