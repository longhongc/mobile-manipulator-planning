/* problem.cpp
 *
 * Author: Chang-Hong Chen
 * Email: longhongc@gmail.com
 */

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <stdexcept>

#include "problem.h"

Problem::Problem(std::string obstacles_file) {
  this->load_workspace(obstacles_file);
  std::cout << "\033[3mLoading complete\033[0m" << std::endl;
}

void Problem::load_workspace(std::string obstacles_file) {
  std::cout << "\033[3mLoading " <<
    obstacles_file << "\033[0m" << std::endl;

  std::ifstream file{obstacles_file};
  std::string line;
  int line_count = 0;

  if (file.is_open()) {
    while (std::getline(file, line)) {
      line_count++;
      // The first line is the total number of obstacles
      if (line_count == 1) {
        continue;
      }

      std::stringstream ss{line};
      std::string item;
      int item_count = 0;
      double x = 0.0;
      double y = 0.0;
      Radius radius = 0.0;
      // Every line has the form
      // x, y, radius
      while (std::getline(ss, item, ',')) {
        switch (item_count) {
          case 0:
            x = stod(item);
            break;
          case 1:
            y = stod(item);
            break;
          case 2:
            radius = stod(item);
            break;
          default:
            break;
        }

        item_count++;
      }

      Circle circle{std::make_pair(x, y), radius};

      circular_obstacles_.push_back(circle);
    }

  } else {
    throw std::invalid_argument("Bad node file name");
  }
}

bool Problem::valid_coord(Coord2D coord) {
  auto& [x, y] = coord;
  if (x < X_MIN || x > X_MAX ||
      y < Y_MIN || y > Y_MAX) {
    return false;
  }

  return true;
}

bool Problem::check_collision(Coord2D coord, double tolerance) {
  for (auto& obstacle : circular_obstacles_) {
    auto& [center, r] = obstacle;
    if (euclidean_distance(coord, center) <= (r + tolerance)) {
      return true;
    }
  }

  return false;
}

bool Problem::check_collision(
        Coord2D coord1, Coord2D coord2, int points) {
  if (points < 2) {
    throw std::invalid_argument(
        "Line collision checking requires at least 2 points");
  }

  auto intermediate_coords = interpolate(coord1, coord2, points);

  for (int i=0; i < points; ++i) {
    Coord2D curr_coord = intermediate_coords[i];

    // Check each of the interpolated coordinates
    if (this->check_collision(curr_coord)) {
      return true;
    }
  }

  return false;
}

bool Problem::check_collision(
    std::vector<State> traj, int points) {

  for (auto& state : traj) {
    if (robot->checkSelfCollision(state, points)) {
      return true;
    }
  }
  
  bool base_guarantee_safe = true;
  for (auto& state : traj) {
    Coord2D base_pose{state.values[0], state.values[1]}; 
    if (this->check_collision(base_pose, robot->workspace_radius)) {
      base_guarantee_safe = false;
      break;
    }
  }

  if (base_guarantee_safe) {
    return false;
  }

  for (auto& state : traj) {
    auto poses = robot->FK(state);
    auto base = poses[0];
    auto j1 = poses[1];
    auto j2 = poses[2];

    if (this->check_collision(base.position, robot->base_radius)) {
      return true;
    }

    if (this->check_collision(base.position, j1.position, points)) {
      return true;
    }

    if (this->check_collision(j1.position, j2.position, points)) {
      return true;
    }
  }

  return false;
}
