set(PLANNERS_SRC
  src/Graph
  src/Random_planner
  src/Optimal_planner

  include/Graph
  include/Random_planner
  include/Optimal_planner
  include/utils
)

add_library(MotionPlanningDiscretePlanner ${PLANNERS_SRC})
target_include_directories (MotionPlanningDiscretePlanner PUBLIC include)
