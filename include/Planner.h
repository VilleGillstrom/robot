#pragma once

#include <glm/glm.hpp>
#include <deque>
#include "Perception.h"
#include "Cartographer.h"
#include "Frontier.h"

class Planner {

public:
    typedef glm::ivec2 point;


    enum mark_type {
        NONE,
        MAP_OPEN_LIST,
        MAP_CLOSE_LIST,
        FRONTIER_OPEN_LIST,
        FRONTIER_CLOSE_LIST
    };



    Planner(Cartographer &cartographer) : mCartographer(cartographer) {

    }
    std::vector<glm::ivec2> ComputePath();
    std::vector<glm::ivec2> ComputePathToCell(glm::ivec2 cell);

    Frontier SelectFrontier();


    void FindFrontiers(std::vector<Frontier> &frontiers);
    point GetRobotPoint() const;
    bool HasOpenSpaceNeighbor(const point &v) const;
    void SetPerception(const std::shared_ptr<Perception> &perception);

private:
    std::shared_ptr<Perception> mPerception;
    Cartographer &mCartographer;

    /** Contains the marks for different points(reset each time) */
    std::vector<std::vector<mark_type>> marks;


    void SetMark(const glm::ivec2 &point, mark_type type);

    mark_type GetMark(const glm::ivec2 &point) const;

    bool IsFrontierPoint(const point &p) const;

    int TwoDtoOneD(int row, int col, int colmax) const {
        return row + col * colmax;
    }

    void ConstructPath(const std::vector<std::vector<glm::ivec2>>& cameFrom, const glm::ivec2& end, std::vector<glm::ivec2>& outPath);
};


