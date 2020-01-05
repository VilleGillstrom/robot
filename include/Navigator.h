#pragma once

#include <glm/glm.hpp>
#include <deque>
#include "Perception.h"
#include "Cartographer.h"

class Navigator {

public:
    typedef glm::ivec2 point;


    enum mark_type {
        NONE,
        MAP_OPEN_LIST,
        MAP_CLOSE_LIST,
        FRONTIER_OPEN_LIST,
        FRONTIER_CLOSE_LIST
    };

    struct marked_point {
        glm::ivec2 point;
        mark_type mark;
    };

    Navigator(Cartographer &cartographer) : mCartographer(cartographer) {

    }

    void SetMark(const glm::ivec2 &point, mark_type type) {
        marks[point.y][point.x] = type;
    }

    mark_type GetMark(const glm::ivec2 &point) const {
        return marks[point.y][point.x];
    }



    void FindFrontiers(std::vector<std::vector<point>>& frontiers) {
        marks = std::vector<std::vector<mark_type>>(mCartographer.MapHeight(),
                                                    std::vector<mark_type>(mCartographer.MapWidth(), mark_type::NONE));

        glm::dvec3 loc = mPerception->GetLaserLocation();
        point robot_point = mCartographer.WorldLocationToCell(loc.x, loc.y);

        std::deque<point> queue_map;

        SetMark(robot_point, mark_type::MAP_OPEN_LIST);
        queue_map.push_back(robot_point);

        while (!queue_map.empty()) {
            point p = queue_map.front();
            queue_map.pop_back();

            if (GetMark(p) == mark_type::MAP_CLOSE_LIST)
                continue;

            if (IsFrontierPoint(p)) {
                std::deque<point> queue_frontier;
                std::vector<point> new_frontier;

                queue_frontier.push_back(p);
                SetMark(p, mark_type::FRONTIER_OPEN_LIST);

                while (!queue_frontier.empty()) {
                    point q = queue_frontier.front();
                    queue_frontier.pop_back();

                    if (GetMark(q) == mark_type::MAP_CLOSE_LIST || GetMark(q) == mark_type::FRONTIER_CLOSE_LIST) {
                        continue;
                    }

                    if (IsFrontierPoint(q)) {
                        new_frontier.push_back(q);
                        std::vector<point> adjs = mCartographer.GetAdjacent(q);
                        for (point w : adjs) {
                            if (GetMark(w) == mark_type::NONE || GetMark(w) == mark_type::MAP_OPEN_LIST) {
                                queue_frontier.push_back(w);
                                SetMark(w, mark_type::FRONTIER_OPEN_LIST);
                            }
                        }
                    }

                    SetMark(q, mark_type::FRONTIER_CLOSE_LIST);
                }
                //SAVE DATA NEW FRONTIER
                frontiers.push_back(new_frontier);

               for(point g : new_frontier) {
                   SetMark(g, mark_type::MAP_CLOSE_LIST);
               }

            }

            std::vector<point> adjs = mCartographer.GetAdjacent(p);
            for (point v : adjs) {
                if (!(GetMark(v) == mark_type::MAP_OPEN_LIST || GetMark(v) == mark_type::MAP_CLOSE_LIST) && HasOpenSpaceNeighbor(v)) {
                    queue_map.push_front(v);
                    SetMark(v, mark_type::MAP_OPEN_LIST);
                }
            }
            SetMark(p, mark_type::MAP_CLOSE_LIST);
        }
    }

    bool HasOpenSpaceNeighbor(const point &v) const {
        std::vector<point> neighbors = mCartographer.GetAdjacent(v);
        for (point n : neighbors) {
            if (mCartographer.GetProbabilityEmpty(n) > 0.5) {
                return true;
            }
        }
        return false;
    }


    void SetPerception(const std::shared_ptr<Perception> &perception) {
        mPerception = perception;
    }

private:
    std::shared_ptr<Perception> mPerception;
    Cartographer &mCartographer;

    /** Contains the marks for different points(reset each time) */
    std::vector<std::vector<mark_type>> marks;

    bool IsFrontierPoint(const point &p) const {
        //WRONG
        return GetMark(p) == FRONTIER_CLOSE_LIST || GetMark(p) == FRONTIER_OPEN_LIST;
    }


};


