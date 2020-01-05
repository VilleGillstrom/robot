//
// Created by twentyletters on 2020-01-05.
//

#include "include/Frontier.h"

Frontier::Frontier() : Frontier(std::vector<glm::ivec2>()){

}

Frontier::Frontier(std::vector<glm::ivec2> frontier_cells) : cells(std::move(frontier_cells)) {
}
