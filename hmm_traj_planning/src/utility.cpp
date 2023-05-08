/* utility.cpp
 *
 * Author: Chang-Hong Chen
 * Email: longhongc@gmail.com
 */

#include <iostream>

#include "utility.h"

double wrap_angle(double angle) {
  auto wrapped = fmod(angle, (2 * M_PI));
  if (wrapped < 0) {
    return 2 * M_PI + wrapped;
  }
  return wrapped;
}

double euclidean_distance(Coord2D coord1, Coord2D coord2) {
  auto& [x1, y1] = coord1;
  auto& [x2, y2] = coord2;
  return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

double euclidean_distance(std::vector<double> v1, std::vector<double> v2) {
  assert(v1.size() == v2.size());
  double total = 0.0;
  for (int i=0; i < v1.size(); ++i) {
    total += pow(v1[i] - v2[i], 2);
  }
  return sqrt(total);
}

std::vector<Coord2D> interpolate(Coord2D coord1, Coord2D coord2, int points) {
  auto& [x1, y1] = coord1;
  auto& [x2, y2] = coord2;

  double x_step = (x2 - x1) / (points - 1);
  double y_step = (y2 - y1) / (points - 1);

  std::vector<Coord2D> intermediate_coords;
  for (int i=0; i < points ; ++i) {
    auto x_i = x1 + x_step * i;
    auto y_i = y1 + y_step * i;
    intermediate_coords.push_back({x_i, y_i});
  }

  return intermediate_coords;
}

void print_pose(Pose p) {
  auto& [x, y] = p.position;
  auto t = p.angle;

  std::cout << "Pose: [" << x  <<
    ", " << y << ", " << t << "]" << std::endl;
}
