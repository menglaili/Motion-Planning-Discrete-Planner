#include "Random_planner.h"

using namespace std;


vector<pint> RandomPlanner::search(const vector<vector<char>> & graph, const pint& robot_pose, const pint& goal_pose, const int& msn) {
    const string name = "Random Planner: " + print_poses(robot_pose, goal_pose);
    Graph world_state(graph, true);
    if (!world_state.is_None(robot_pose, goal_pose)) { 
        print_none(name);
        return {};
    }
    int rInd = world_state.xy2ind(robot_pose), gInd = world_state.xy2ind(goal_pose);
    if (world_state.heuristic(rInd, gInd) > msn) { // Manhattan distance > max_step_number, no way
        print_none(name);
        return {};
    }
    max_step_number = msn; max_memory = round(sqrt(max_step_number));
    vector<pint> results; results.push_back(world_state.ind2xy(rInd));
    int curInd = rInd; states_queue.push(rInd); states_set.insert(rInd);
    while (results.size() < max_step_number) {
        int nextInd = select_move(curInd, world_state);
        if (nextInd < 0) { // start and end is blocked by wall
            print_none(name);
            return {};
        }
        results.push_back(world_state.ind2xy(nextInd));
        if (nextInd == gInd) {
            print_path(results, name);
            return results;
        }
        if (states_set.size() == max_memory) {
            int tmp = states_queue.front(); states_queue.pop();
            states_set.erase(tmp);
        } 
        states_queue.push(nextInd); states_set.insert(nextInd);
        curInd = nextInd;
    }
    print_none(name);
    return {};
}

int RandomPlanner::random_unint(int maxV) {   
    random_device rd;
    mt19937 gen(rd());
    static uniform_int_distribution<> u(0, maxV);
    int ind = u(gen); 
    return min(ind, maxV);
}

int RandomPlanner::select_move(const int& curInd, Graph& world_state) {
    vector<int> children = world_state.find_children(curInd);
    if (children.empty()) return -1;
    vector<int> selected;
    for (const auto& child : children) {
        if (!states_set.count(child)) {
            selected.push_back(child);
        }
    }
    if (selected.empty()) {
        int ind = random_unint(children.size()-1);
        return children[ind];
    } else {
        int ind = random_unint(selected.size()-1);
        return selected[ind];
    }
    return -1;
}
