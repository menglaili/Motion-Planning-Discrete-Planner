#ifndef OPTIMAL_H
#define OPTIMAL_H

#include "utils.h"
#include "Graph.h"
#include <string>
#include <boost/heap/fibonacci_heap.hpp>
#include <vector>
#include <iostream>


class OptimalPlanner {
public:    
    std::vector<pint> search(const std::vector<std::vector<char>> & graph, const pint& robot_pose, const pint& goal_pose);

private:
    int find_fvalue(const int& rInd, const int& gInd, const std::vector<int>& gvalues, Graph& world_state);
};

#endif