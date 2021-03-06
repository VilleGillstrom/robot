#include "include/Planner.h"
#include <algorithm>


std::vector<glm::ivec2> Planner::SelectPath() {
    std::vector<Frontier> frontiers = OrderedFindFrontiers();
    std::vector<glm::ivec2> outpath;

    for (const auto &frontier : frontiers) {
        glm::ivec2 centroid = frontier.GetCentroid();
        // If centroid same as  last and or we 
        if (centroid == lastCentroid) {
            //Possbly stuck, in center, try to find a path to cell in frontier
            std::vector<glm::ivec2> cellsInCentroid = frontier.GetCells();
            glm::ivec2 cellInFrontier = cellsInCentroid[cellsInCentroid.size() / 2];
            glm::ivec2 cellRobot = mCartographer.RobotLocation();
            glm::ivec2 cellMid((int) ((float) (cellInFrontier.x + cellRobot.x) / 2.0),
                               (int) ((float) (cellInFrontier.y + cellRobot.y) / 2.0));

            if (ComputePathToCell(cellMid, outpath)) {
                lastCentroid = cellMid;
                break;
            } else {
                //Check the next frontier
                continue;
            }
        }

        // If can find a path to centroid
        if (ComputePathToCell(centroid, outpath)) {
            lastCentroid = centroid;
            break;
        }
    }

    return outpath;
}

//A Star from wikipedias psuedocode https://en.wikipedia.org/wiki/A*_search_algorithm
bool Planner::ComputePathToCell(glm::ivec2 cell, std::vector<glm::ivec2> &path) {

    auto map = mCartographer.GetProbablityGrid();
    glm::ivec2 start = GetRobotPoint();
    glm::ivec2 goal = cell;

    //heurisitcs is the distance between two cells
    auto h = [goal](glm::ivec2 n) { return glm::distance(glm::vec3(n.x, n.y, 0), glm::vec3(goal.x, goal.y, 0)); };
    Cartographer &cg = this->mCartographer;
    auto d = [&cg](glm::ivec2 current, glm::ivec2 neighbor) {
        const float distanceToCell = glm::distance(cg.CellToWorldLocation({current.x, current.y}), cg.CellToWorldLocation({neighbor.x, neighbor.y}));
        const float occupiedWeight = (8 - cg.GetEmptyAdjacent(neighbor).size()) * 100; //Hate if neighbor has occuupied adjacents
        const float reactWeight = cg.GetDangerGrid().GetCellValue(neighbor) * 20;
        return distanceToCell + occupiedWeight + reactWeight;
    };

    int mapHeight = mCartographer.MapHeight();
    int mapWidth = mCartographer.MapWidth();

    std::vector<glm::ivec2> openSet;
    std::vector<std::vector<glm::ivec2>> cameFrom(mapHeight, std::vector<glm::ivec2>(mapWidth, {-1, -1}));
    std::vector<std::vector<float>> gScore(mapHeight, std::vector<float>(mapWidth, std::numeric_limits<float>::max()));
    std::vector<std::vector<float>> fScore(mapHeight, std::vector<float>(mapWidth, std::numeric_limits<float>::max()));


    openSet.push_back(start);
    gScore[start.x][start.y] = 0;
    fScore[start.x][start.y] = h(start);


    while (!openSet.empty()) {
        float min_value = std::numeric_limits<float>::max();
        glm::ivec2 current;
        for (const glm::ivec2 &cell : openSet) {
            int row = cell.x;
            int col = cell.y;
            float test_score = fScore[row][col];
            if (test_score < min_value) {
                min_value = test_score;
                current = {row, col};
            }
        }


        if (current == goal) {
            ConstructPath(cameFrom, current, path);
            return true;
        }

        openSet.erase(std::remove(openSet.begin(), openSet.end(), current), openSet.end());

        std::vector<glm::ivec2> neighbors = GetPossibleNodesAStar(current);
        for (auto neighbor : neighbors) {
            // d(current,neighbor) is the weight of the edge from current to neighbor
            // tentative_gScore is the distance from start to the neighbor through current
            float tentative_gScore = gScore[current.x][current.y] + d(current, neighbor);
            float neighbor_score = gScore[neighbor.x][neighbor.y];
            if (tentative_gScore < neighbor_score) {
                // This path to neighbor is better than any previous one. Record it!
                cameFrom[neighbor.x][neighbor.y] = current;;
                gScore[neighbor.x][neighbor.y] = tentative_gScore;
                fScore[neighbor.x][neighbor.y] = tentative_gScore + h(neighbor);
                if (std::find(openSet.begin(), openSet.end(), neighbor) == openSet.end()) {
                    openSet.push_back(neighbor);
                }
            }
        }
    }
    std::cout << "Unable to find path to cell: " << cell.x << ", " << cell.y << ")" << std::endl;
    return false;
}

Planner::mark_type Planner::GetMark(const glm::ivec2 &point) const {
    //glm::ivec2 cell = mCartographer.PointToCell(point);
    if (point.x < 0 || point.x >= marks.size())
        return mark_type::NONE;
    if (point.y < 0 || point.y >= marks[0].size())
        return mark_type::NONE;

    return marks[point.x][point.y];
}

void Planner::SetMark(const glm::ivec2 &point, Planner::mark_type type) {
    marks[point.x][point.y] = type;
}

bool Planner::IsFrontierPoint(const Planner::point &p) const {
    bool isFrontier = mCartographer.IsUnknown(p) && HasOpenSpaceNeighbor(p);
    return isFrontier;
}

//https://arxiv.org/ftp/arxiv/papers/1806/1806.03581.pdf
void Planner::FindFrontiers(std::vector<Frontier> &frontiers) {
    marks = std::vector<std::vector<mark_type>>(mCartographer.MapHeight(),
                                                std::vector<mark_type>(mCartographer.MapWidth(), mark_type::NONE));

    point robot_point = GetRobotPoint();

    std::deque<point> queue_map;

    SetMark(robot_point, mark_type::MAP_OPEN_LIST);
    queue_map.push_back(robot_point);

    while (!queue_map.empty()) {
        point p = queue_map.front();
        queue_map.pop_front();

        if (GetMark(p) == mark_type::MAP_CLOSE_LIST)
            continue;

        if (IsFrontierPoint(p)) {
            std::deque<point> queue_frontier;
            std::vector<point> new_frontier;

            queue_frontier.push_back(p);
            SetMark(p, mark_type::FRONTIER_OPEN_LIST);

            while (!queue_frontier.empty()) {
                point q = queue_frontier.front();
                queue_frontier.pop_front();

                if (GetMark(q) == mark_type::MAP_CLOSE_LIST || GetMark(q) == mark_type::FRONTIER_CLOSE_LIST) {
                    continue;
                }

                if (IsFrontierPoint(q)) {
                    new_frontier.push_back(q);
                    std::vector<point> adjs = mCartographer.GetAdjacent(q);
                    for (point w : adjs) {
                        mark_type w_mark = GetMark(w);
                        if (w_mark != mark_type::FRONTIER_OPEN_LIST && w_mark != mark_type::FRONTIER_CLOSE_LIST &&
                            w_mark != mark_type::MAP_CLOSE_LIST) {
                            queue_frontier.push_back(w);
                            SetMark(w, mark_type::FRONTIER_OPEN_LIST);
                        }
                    }
                }

                SetMark(q, mark_type::FRONTIER_CLOSE_LIST);
            }
            //SAVE DATA NEW FRONTIER
            frontiers.push_back(Frontier(new_frontier));

            for (point g : new_frontier) {
                SetMark(g, mark_type::MAP_CLOSE_LIST);
            }

        }

        std::vector<point> adjs = mCartographer.GetAdjacent(p);
        for (point v : adjs) {
            mark_type v_mark = GetMark(v);
            if ((v_mark != mark_type::MAP_OPEN_LIST) && (v_mark != mark_type::MAP_CLOSE_LIST) &&
                HasOpenSpaceNeighbor(v)) { //
                queue_map.push_back(v);
                SetMark(v, mark_type::MAP_OPEN_LIST);
            }
        }
        SetMark(p, mark_type::MAP_CLOSE_LIST);
    }
}

std::vector<glm::ivec2> Planner::GetPossibleNodesAStar(const Planner::point &q) const {
    std::vector<glm::ivec2> out;
    const std::vector<glm::ivec2> &neighbors = mCartographer.GetNeighborsLessThan(q, 0.51);
    return neighbors;
}

Planner::point Planner::GetRobotPoint() const {
    glm::dvec3 loc = mCartographer.RobotLocation();
    point robot_point = mCartographer.WorldLocationToCell(loc.x, loc.y);
    return robot_point;
}

bool Planner::HasOpenSpaceNeighbor(const Planner::point &v) const {
    std::vector<point> neighbors = mCartographer.GetAdjacent(v);
    for (point n : neighbors) {
        if (mCartographer.GetProbabilityEmpty(n) > 0.50 && !mCartographer.IsUnknown(n)) {
            return true;
        }
    }
    return false;
}

std::vector<Frontier> Planner::OrderedFindFrontiers() {
    std::vector<Frontier> frontiers;
    std::vector<Frontier> ordered_frontiers;
    FindFrontiers(frontiers);

    unsigned int numfrontiers = frontiers.size();
    //Order frontiers with most unexplored cells
    for (int i = 0; i < numfrontiers; ++i) {
        Frontier selectedFrontier;
        int idx = -1;
        for (int frontierIdx = 0; frontierIdx < frontiers.size(); frontierIdx++) {
            const auto &frontier = frontiers[frontierIdx];
            if (frontier.GetCells().size() > selectedFrontier.GetCells().size()) {
                selectedFrontier = frontier;
                idx = frontierIdx;
            }

        }
        frontiers.erase(frontiers.begin() + idx);
        ordered_frontiers.push_back(selectedFrontier);
    }

    return ordered_frontiers;
}



void Planner::ConstructPath(const std::vector<std::vector<glm::ivec2>> &cameFrom, const glm::ivec2 &end,
                            std::vector<glm::ivec2> &out_path) {
    glm::ivec2 current = end;
    out_path.clear();
    out_path.push_back(end);
    //What
    while (current != cameFrom[current.x][current.y] && cameFrom[current.x][current.y].x != -1) {
        current = cameFrom[current.x][current.y];
        out_path.push_back(current);
    }
    std::reverse(out_path.begin(), out_path.end());
}

Planner::Planner(Cartographer &cartographer) : mCartographer(cartographer) {}

