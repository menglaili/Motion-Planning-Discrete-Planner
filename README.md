# Motion Planning Discrete Planner

## Run tests:
```
sudo apt-get install libboost-all-dev
mkdir build
cd build
cmake .. 
make
./test/test_planners
```

## File architectures:
- planners
  - include
    - Graph.h: header file for class Graph
    - Optimal_planner.h: header file for class Optimal planner
    - Random_planner.h: header file for class random planner
    - utils.h: header file for defined type
  - src
    - Graph.cpp: source file for class Graph
    - Optimal_planner.cpp: source file for class Optimal planner
    - Random_planner.cpp: source file for class random planner
- test
  - main.cpp: main file for testing both of the planning algorithms
  
## Implementation:
### Graph representation
Convert each 2D x-y coordinates to 1D index. Each proper index is considered as one node.
### Optimal planner
Implement A* algorithm. Use fibonacci heap from C++ boost library to store f-values of each node. 
![image](<img width="150" height="150" src="https://github.com/menglaili/Motion-Planning-Discrete-Planner/blob/master/Astar.jpg"/>)
### Random planner
Use queue to maintain max_step_number of nodes which are also stored in another unordered_set. At each move, check whether children nodes are in unordered_set.

## Complecity analysis:
Number of free nodes in the 2D grids: |V|, Number of edges between free nodes: |E|
### Optimal planner
Time: O(|V|) + O(|V|log|V|) + O(|V|) + O(1)O(|E|) = O(|E| + |V|log|V|)
space: O(|V|^2)
### Random planner
Time: O(max_step_number)
space: O(max_step_number^0.5)



