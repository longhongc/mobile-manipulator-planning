#include <iostream>
#include <vector>
#include <memory>
#include <numeric>
#include <cmath>

#include "problem.h"
#include "rrt_cspace.h"

void run_trial(
    double& search_time, 
    double& search_node,
    double& path_time) {
  State start_state{
    {0.0, 0.0, 0.0, 0.25 * M_PI}, {0.0, 0.0, 0.0, 0.0}, 0.0};
    // {0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}, 0.0};
  // Coord2D goal_coord{30, 30};
  Coord2D goal_coord{16, -2};

  std::string obstacles_file =
    "./data/obstacles.txt";

  std::shared_ptr<Problem> problem_ptr =
    std::make_shared<Problem>(obstacles_file);

  RRTCSpace solver(start_state, goal_coord, problem_ptr);
  auto path = solver.solve();

  // solver.print_path();
  search_time = solver.search_time();
  search_node = solver.search_node();
  path_time = solver.path_time();
}

double calc_average(std::vector<double> vec) {
  double sum = std::accumulate(vec.begin(), vec.end(), 0.0);
  return sum / vec.size();
}

double calc_std(double average, std::vector<double> vec) {
  double variance = 0.0;
  for (auto num : vec) {
    variance += pow(num - average, 2);
  }

  return sqrt(variance / vec.size());
}

int main(int argc, char** argv) {
  std::vector<double> search_time_vec; 
  std::vector<double> search_node_vec;
  std::vector<double> path_time_vec;

  int trials = 50;
  for (int i=0; i<trials; ++i) {
    double search_time;
    double search_node;
    double path_time;
    run_trial(search_time, search_node, path_time);
    search_time_vec.push_back(search_time);
    search_node_vec.push_back(search_node);
    path_time_vec.push_back(path_time);
  }

  auto st_average = calc_average(search_time_vec);
  auto st_std = calc_std(st_average, search_time_vec);
  std::cout << "Search time: " << std::endl;
  std::cout << "Avg: " << st_average << ", std: " << 
    st_std << std::endl;

  auto sn_average = calc_average(search_node_vec);
  auto sn_std = calc_std(sn_average, search_node_vec);
  std::cout << "Search node: " << std::endl;
  std::cout << "Avg: " << sn_average << ", std: " << 
    sn_std << std::endl;

  auto pt_average = calc_average(path_time_vec);
  auto pt_std = calc_std(pt_average, path_time_vec);
  std::cout << "Path time: " << std::endl;
  std::cout << "Avg: " << pt_average << ", std: " << 
    pt_std << std::endl;

  return 0;
}
