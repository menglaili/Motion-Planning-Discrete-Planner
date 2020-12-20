#ifndef  GRAPH_H
#define GRAPH_H

#include "utils.h"
#include <string>
#include <queue>
#include <vector>
#include <random>
#include <ctime>
#include <iostream>
#include <unordered_set>


class Graph {    
public:
    std::vector<std::vector<char>> grids; // grids input
    int rowN, colN;            // row number, column number
    std::vector<int> islands;                 // store different connected components
    
    Graph() = delete;
    Graph(Graph& rhs) = delete;
    Graph& operator=(const Graph& rhs) = delete;
    Graph(const std::vector<std::vector<char>> &g, const bool& print_sign);

    // from 1d index representation to 2d (x, y) representation
    pint ind2xy(const int & ind);
    
    // from 2d (x, y) representation to 1d index representation
    int xy2ind(const int & x, const int & y);
    
    int xy2ind(const pint & p);
    
    //if false then return no path
    bool is_None(const pint& robot_pose, const pint& goal_pose);
    
    //add connection check
    bool is_None_connect(const pint& robot_pose, const pint& goal_pose);
    
    // find the children states
    std::vector<int> find_children(const int& rootInd);
    
    // calculate the Manhattan distance as heuristic between current state and goal state
    int heuristic (const int& indNow, const int& indGoal);
    
private:  
    std::vector<int> rDir = {1, -1, 0, 0}, cDir = {0, 0, 1, -1};
    // depth-first-search
    void dfs(const int & rootX, const int & rootY, const int& mark);
    // mark connected components in the grids
    void find_connected_component();
    
    // check whether the input grids is empty
    bool check_empty();
    
    // check whether the state is in boundary and not obstacles
    bool isvaild_state(const pint& xy);
    
    // check whether two states belong to same connected component
    bool is_connected(const pint& xy1, const pint& xy2);
    
    // print the grids
    void print_grids(const bool& print_sign);
};

void print_path(const std::vector<pint>& results, const std::string& name);


void print_none(const std::string & name);

std::string print_poses(const pint& robot_pose, const pint& goal_pose);

#endif