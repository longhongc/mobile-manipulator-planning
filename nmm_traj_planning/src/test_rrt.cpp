#include <iostream>
#include <memory>

#include "problem.h"
#include "rrt_cspace.h"

int main(int argc, char** argv) {
  State start_state{
    {0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}, 0.0};
  // Coord2D goal_coord{30, 30};
  Coord2D goal_coord{16, -2};

  std::string obstacles_file =
    "./data/obstacles.txt";

  std::shared_ptr<Problem> problem_ptr =
    std::make_shared<Problem>(obstacles_file);

  RRTCSpace solver(start_state, goal_coord, problem_ptr);
  auto path = solver.solve();
  solver.print_path();

  std::string path_file = "./path.txt";
  solver.save_path_to_file(path_file);

  return 0;
}

