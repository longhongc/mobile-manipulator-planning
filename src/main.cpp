/* main.cpp
 *
 * Author: Chang-Hong Chen
 * Email: longhongc@gmail.com
 */

#include <iostream>
#include <memory>

#include "problem.h"
#include "rrt_cspace.h"

int main(int argc, char** argv) {

  State start_state{{0, 0}, {0, 0}};
  Coord2D goal_coord{30, 30};

  std::shared_ptr<Problem> problem_ptr =
    std::make_shared<Problem>();

  RRTCSpace solver(start_state, goal_coord, problem_ptr);
  auto path = solver.solve();
  solver.print_path();

  // std::string result_dir = "results/Problem" + problem_number;
  // std::filesystem::create_directories(result_dir);
  std::string path_file = "./path.txt";
  solver.save_path_to_file(path_file);
  return 0;
}
