#pragma once

#include <utility>
#include <vector>
#include <glm/glm.hpp>
#include <QtGui/QColor>

class Frontier {
public:
    Frontier();
    Frontier(std::vector<glm::ivec2> frontier_cells);

    const std::vector<glm::ivec2>& GetCells() const {
        return cells;
    }

    const glm::ivec2 GetCentroid() const {
        if(cells.size() == 0) {
            return glm::ivec2(0);
        }

        glm::ivec2 sum(0);
        for (const glm::ivec2 &cell : cells) {
            sum +=cell;
        }
        glm::ivec2 centroid = sum / (int) cells.size();
        return centroid;
    }
private:
    std::vector<glm::ivec2> cells;

};

