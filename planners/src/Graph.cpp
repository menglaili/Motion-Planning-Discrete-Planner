#include "Graph.h"


using namespace std;

Graph::Graph(const vector<vector<char>> &g, const bool& print_sign) : grids(g) {
    if (!check_empty()) {
        rowN = grids.size(); colN = grids[0].size();
        print_grids(print_sign);
    } else {
        if (print_sign) cout << "Empty grids" << endl;
    }
}

// from 1d index representation to 2d (x, y) representation
pint Graph::ind2xy(const int & ind)  { 
    int x = ind / colN, y = ind % rowN;
    return make_pair(x, y);
}

// from 2d (x, y) representation to 1d index representation
int Graph::xy2ind(const int & x, const int & y) { 
    if (x >= rowN || x < 0 || y >= colN || y < 0) {
        cout << "out of boundary at xy2ind! " << x << " " << y << endl;
    }
    return colN * x + y;
}

int Graph::xy2ind(const pint & p) {  // pair p = (x, y)
    int x = p.first, y = p.second;
    return xy2ind(x, y);
}

//if false then return no path
bool Graph::is_None(const pint& robot_pose, const pint& goal_pose) {
    if (check_empty() || !isvaild_state(robot_pose) || \
    !isvaild_state(goal_pose)) {
        return false;
    }
    return true;
}

//add connection check
bool Graph::is_None_connect(const pint& robot_pose, const pint& goal_pose) {
    if (check_empty() || !isvaild_state(robot_pose) || \
    !isvaild_state(goal_pose) || !is_connected(robot_pose, goal_pose)) {
        return false;
    }
    return true;
}

// find the children states
vector<int> Graph::find_children(const int& rootInd) {   
    pint rootXy = ind2xy(rootInd); int rootX = rootXy.first, rootY = rootXy.second;
    if (grids[rootX][rootY] == '1') {
        cout << "find children of a wall at find_children! " << rootInd << endl;
    }
    vector<int> children; 
    for (int i = 0; i < 4; ++i) {
        if (rootX + rDir[i] < rowN && rootX + rDir[i] >= 0 && rootY + cDir[i] < colN && \
                        rootY + cDir[i] >= 0 && grids[rootX + rDir[i]][rootY + cDir[i]] != '1') {
            int resInd = xy2ind(rootX + rDir[i], rootY + cDir[i]);
            children.push_back(resInd);
        }
    }
    return children;
}

// calculate the Manhattan distance as heuristic between current state and goal state
int Graph::heuristic (const int& indNow, const int& indGoal) {   
    pint xyNow = ind2xy(indNow); int xNow = xyNow.first, yNow = xyNow.second;
    pint xyGoal = ind2xy(indGoal); int xGoal = xyGoal.first, yGoal = xyGoal.second;
    return abs(xNow - xGoal) + abs(yNow - yGoal);
}


// depth-first-search
void Graph::dfs(const int & rootX, const int & rootY, const int& mark) {  // only 1 is block
    if (rootX < rowN && rootX >= 0 && rootY < colN && rootY >= 0 && grids[rootX][rootY] == '0') {
        grids[rootX][rootY] = 'v'; // change visited node to 'v'
        islands[xy2ind(rootX, rootY)] = mark; 
        for (int i = 0; i < 4; ++i) dfs(rootX + rDir[i], rootY + cDir[i], mark);
    }
}
// mark connected components in the grids
void Graph::find_connected_component()  {
    int mark = 1;
    for (int i = 0; i < rowN; ++i) {
        for (int j = 0; j < colN; ++j) {
            dfs(i, j, mark++); 
        }
    }
}


// check whether the input grids is empty
bool Graph::check_empty() {
    if (grids.empty() || grids[0].empty()) return true;
    return false;
}

// check whether the state is in boundary and not obstacles
bool Graph::isvaild_state(const pint& xy) {
    int x = xy.first, y = xy.second;
    if (x >= rowN || x < 0 || y >= colN || y < 0 || grids[x][y] == '1') return false;
    return true;
}

// check whether two states belong to same connected component
bool Graph::is_connected(const pint& xy1, const pint& xy2) {
    islands = vector<int> (rowN * colN, 0);
    find_connected_component(); 
    int ind1 = xy2ind(xy1), ind2 = xy2ind(xy2);
    if (islands[ind1] != islands[ind2]) return false;
    return true;
}

void Graph::print_grids(const bool& print_sign) {
    if (!print_sign) return;
    if (rowN * colN > 100) {
        cout << "Grids larger than 10 * 10 is not showing" << endl;
        return;
    }
    for(int i = 0; i < rowN; ++i) {
        for (int j = 0; j < colN; ++j) {
            cout << grids[i][j] << ' ';
        }
        cout << endl;
    }
}

void print_path(const vector<pint>& results, const string& name) {
    cout << name << endl; cout << '[';
    for (int i = 0; i < results.size(); ++i) {
        pint path = results[i];
        if (i != results.size()-1) {
            cout << '(' << path.first << ", " << path.second << "), ";
        } else {
            cout << '(' << path.first << ", " << path.second << ")]" << endl;
        }
    }
}

void print_none(const std::string& name) {
    cout << name << endl;
    cout << "No path has been found." << endl;
}

string print_poses(const pint& robot_pose, const pint& goal_pose) {
    string x1 = to_string(robot_pose.first), y1 = to_string(goal_pose.first), x2 = to_string(robot_pose.second), y2 = to_string(goal_pose.second);
    return "robot_pose (" + x1 + ", " + y1 + ") " + "goal_pose (" + x2 +  ", " + y2 + ')'; 
}