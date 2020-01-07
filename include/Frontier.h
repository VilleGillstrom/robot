#pragma once

#include <utility>
#include <vector>
#include <glm/glm.hpp>
#include <QtGui/QColor>


/**
 * Represents a frontier
 */
class Frontier {
public:
    Frontier();
    explicit Frontier(std::vector<glm::ivec2> frontier_cells);

    // Get all cells in frontier
    const std::vector<glm::ivec2>& GetCells() const;

    // Get the centroid
    glm::ivec2 GetCentroid() const;
private:

    //Cells in frontier
    std::vector<glm::ivec2> cells;
};

