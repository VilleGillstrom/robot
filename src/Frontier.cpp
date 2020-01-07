//
// Created by twentyletters on 2020-01-05.
//

#include "include/Frontier.h"

Frontier::Frontier() : Frontier(std::vector<glm::ivec2>()){

}

Frontier::Frontier(std::vector<glm::ivec2> frontier_cells) : cells(std::move(frontier_cells)) {
}

glm::ivec2 Frontier::GetCentroid() const {
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

const std::vector<glm::ivec2> &Frontier::GetCells() const {
    return cells;
}
