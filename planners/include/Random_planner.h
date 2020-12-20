#ifndef RANDOMP_H
#define RANDOMP_H

#include "utils.h"
#include "Graph.h"
#include <string>
#include <queue>
#include <vector>
#include <random>
#include <iostream>
#include <unordered_set>


class RandomPlanner {
public:
    std::vector<pint> search(const std::vector<std::vector<char>> & graph, const pint& robot_pose, const pint& goal_pose, const int& msn);
    
private:
    int max_step_number;
    int max_memory;
    std::queue<int> states_queue;
    std::unordered_set<int> states_set;
    
    // find random int
    int random_unint(int max);
    
    // select next states
    int select_move(const int& curInd, Graph& world_state);
};

#endif
