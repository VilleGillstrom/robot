#pragma once

#include <glm/glm.hpp>
#include <deque>
#include <utility>
#include "Perception.h"
#include "Cartographer.h"
#include <memory>
#include "Frontier.h"

/**
 * Responsible for selecting a path to take when exploring based on cartographer's world
 * representation
 */

class Planner {

public:
    typedef glm::ivec2 point;

    // Used by Wavefront Frontier Detector  (https://arxiv.org/ftp/arxiv/papers/1806/1806.03581.pdf)
    enum mark_type {
        NONE,
        MAP_OPEN_LIST,
        MAP_CLOSE_LIST,
        FRONTIER_OPEN_LIST,
        FRONTIER_CLOSE_LIST
    };

    Planner(Cartographer &cartographer);
    std::vector<glm::ivec2> SelectPath();  // Select a path based on cartographer

    std::vector<Frontier> OrderedFindFrontiers();   //Find frontier and order them, first 1 should be explored first
    void FindFrontiers(std::vector<Frontier> &frontiers); //find frontiers

private:
    Cartographer &mCartographer;
    glm::ivec2 lastCentroid;    //cell we went to last time

    /* Contains the marks for different points(reset each time) */
    std::vector<std::vector<mark_type>> marks;
    void SetMark(const glm::ivec2 &point, mark_type type);
    mark_type GetMark(const glm::ivec2 &point) const;

    bool IsFrontierPoint(const point &p) const; //Can point be considered frontier ? (is unknown and has empty neighbor)

    /* Construct a path after A* */
    void ConstructPath(const std::vector<std::vector<glm::ivec2>>& cameFrom, const glm::ivec2& end, std::vector<glm::ivec2>& outPath);
    std::vector<glm::ivec2> GetPossibleNodesAStar(const point &q) const; /* Get possible neighbors to move to from point q*/
    bool ComputePathToCell(glm::ivec2 cell, std::vector<glm::ivec2>& path); /* Construct a path using A* to cell */
    point GetRobotPoint() const;    // What cell is the robot in?
    bool HasOpenSpaceNeighbor(const point &cell) const; //does cell have any neighbor that is empty?
};


