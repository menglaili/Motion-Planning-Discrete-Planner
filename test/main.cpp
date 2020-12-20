#include "Optimal_planner.h"
#include "Random_planner.h"
#include <vector>

using namespace std;

vector<vector<char>> gen_rand(const int& rowN, const int& colN, bool rand_sign = true) {
    vector<vector<char>> grids (rowN, vector<char> (colN, '0'));
    if (!rand_sign) return grids;
    random_device rd;
    mt19937 gen(rd());
    static uniform_int_distribution<> u(0, 1);
    for (int i = 0; i < rowN; ++i) {
        for (int j = 0; j < colN; ++j) {
            if (u(gen) != 0) grids[i][j] = '1';
        }
    }
    return grids;
}

void testing() {
    vector<vector<char>> grids0 = {{}};
    vector<vector<char>> grids1 = {{'0', '0', '1', '0', '0', '0'},
                                                    {'0', '0', '1', '0', '0', '0'},
                                                    {'0', '0', '0', '0', '1', '0'},
                                                    {'0', '0', '0', '0', '1', '0'},
                                                    {'0', '0', '1', '1', '1', '0'},
                                                    {'0', '0', '0', '0', '0', '0'}};                             
    vector<vector<char>> grids2 = {{'0', '0', '0', '0', '0', '0'},
                                                    {'0', '1', '0', '1', '0', '0'},
                                                    {'0', '1', '0', '1', '1', '0'},
                                                    {'1', '1', '0', '1', '1', '0'},
                                                    {'1', '1', '0', '1', '1', '0'},
                                                    {'1', '1', '0', '0', '0', '0'}};
    vector<vector<char>> grids3 = {{'0', '0', '1', '0', '0', '0'},
                                                    {'0', '0', '1', '0', '0', '0'},
                                                    {'0', '0', '1', '0', '1', '0'},
                                                    {'0', '0', '1', '0', '1', '0'},
                                                    {'0', '0', '1', '1', '1', '0'},
                                                    {'0', '0', '1', '0', '0', '0'}};
    vector<vector<char>> grids4 = {{'0', '0', '1', '0', '0', '0'},
                                                    {'1', '0', '1', '0', '0', '0'},
                                                    {'0', '1', '0', '0', '1', '0'},
                                                    {'1', '0', '0', '0', '1', '0'},
                                                    {'0', '0', '1', '1', '1', '0'},
                                                    {'0', '0', '0', '1', '0', '1'}};
    vector<vector<char>> grids5 = gen_rand(100, 100, false);
    vector<vector<char>> grids6 = gen_rand(100, 100);
    
    RandomPlanner rp;
    OptimalPlanner op;
    
    // grids0: empty input
    rp.search(grids0, make_pair(2, 0), make_pair(5, 4), 35);
    op.search(grids0, make_pair(2, 0), make_pair(5, 4));
    // grids1: simple input with easy solution
    rp.search(grids1, make_pair(2, 0), make_pair(5, 4), 35);
    op.search(grids1, make_pair(2, 0), make_pair(5, 4));
    rp.search(grids1, make_pair(2, 0), make_pair(5, 4), 4);
    // grids2: simple input with hard solution
    rp.search(grids2, make_pair(2, 0), make_pair(5, 4), 35);
    op.search(grids2, make_pair(2, 0), make_pair(5, 4));
    rp.search(grids2, make_pair(2, 0), make_pair(2, 3), 4);
    // grids3: simple input with no solution
    rp.search(grids3, make_pair(2, 0), make_pair(5, 4), 35);
    op.search(grids3, make_pair(2, 0), make_pair(5, 4));
    rp.search(grids2, make_pair(2, 0), make_pair(5, 4), 4);
    // grids4: start and end points have no children
    rp.search(grids4, make_pair(2, 0), make_pair(5, 4), 35);
    op.search(grids4, make_pair(2, 0), make_pair(5, 4));
    rp.search(grids4, make_pair(2, 0), make_pair(5, 4), 1);
    // grids5: complex input with easy solution
    rp.search(grids5, make_pair(2, 0), make_pair(95, 94), 100);
    op.search(grids5, make_pair(2, 0), make_pair(95, 94));
    // grids6: random complex random input
    rp.search(grids6, make_pair(2, 0), make_pair(95, 94), 100);
    op.search(grids6, make_pair(2, 0), make_pair(95, 94));
}

int main(){
    testing();
}