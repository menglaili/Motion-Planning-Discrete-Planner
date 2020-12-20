#include "Optimal_planner.h"
#include <limits.h>

using namespace std;

// define the priority to pop the minimum
struct compare_pair
{
    bool operator()(const pint& n1, const pint& n2) const 
    {
        return n1.second > n2.second;
    }
};


vector<pint> OptimalPlanner::search(const vector<std::vector<char>> & graph, const pint& robot_pose, const pint& goal_pose) {
    const string name = "Optimal Planner: " + print_poses(robot_pose, goal_pose);
    Graph world_state(graph, false);
    // check whether the grids or states is invaild
    if (!world_state.is_None_connect(robot_pose, goal_pose)){
        print_none(name);
        return {};
    }
    int totN = world_state.rowN * world_state.colN;
    vector<pint> results;
    boost::heap::fibonacci_heap<pint, boost::heap::compare<compare_pair>> OPEN;
    vector<bool> CLOSE (totN, false); // false means state is not in the close set 
    vector<int> Parents (totN, -1),  gvalues (totN, INT_MAX);
    int rInd = world_state.xy2ind(robot_pose), gInd = world_state.xy2ind(goal_pose);
    gvalues[rInd] = 0; Parents[rInd] = rInd;
    OPEN.push(make_pair(rInd, 0)); 
    while (!CLOSE[gInd]) {
        pint pInd = OPEN.top(); OPEN.pop(); int parent = pInd.first;
        CLOSE[parent] = true;
        vector<int> children = world_state.find_children(parent);
        for (const auto& child : children) {
            if (!CLOSE[child]) {
                if (gvalues[child] > gvalues[parent] + 1) {
                    gvalues[child] = gvalues[parent] + 1;
                    Parents[child] = parent;
                    OPEN.push(make_pair(child, find_fvalue(child, gInd, gvalues, world_state)));
                }
            }
        }
    }
    int tmp = gInd;
    while (tmp != rInd) {
        results.push_back(world_state.ind2xy(tmp));
        tmp = Parents[tmp];
    }
    results.push_back(world_state.ind2xy(rInd));
    reverse(results.begin(), results.end());
    print_path(results, name);
    return results;
}

int OptimalPlanner::find_fvalue(const int& rInd, const int& gInd, const vector<int>& gvalues, Graph& world_state) {
    int hvalue = world_state.heuristic(rInd, gInd);
    int gvalue = gvalues[rInd];
    return gvalue + hvalue;
}
