/* utility.h
 *
 * Author: Chang-Hong Chen
 * Email: longhongc@gmail.com
 */

#pragma once

#include <cmath>
#include <cassert>
#include <utility>
#include <vector>

using Coord2D = std::pair<double, double>;

struct Pose {
  Coord2D position;
  double angle;
};

struct State {
  Coord2D base_pose;
  std::vector<double> link_angles;
};

inline double wrap_angle(double angle) {
  auto wrapped = fmod(angle, (2 * M_PI));
  if (wrapped < 0) {
    return 2 * M_PI + wrapped;
  }
  return wrapped;
}

inline double euclidean_distance(Coord2D coord1, Coord2D coord2) {
  auto& [x1, y1] = coord1;
  auto& [x2, y2] = coord2;
  return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

inline double euclidean_distance(std::vector<double> angles1, std::vector<double> angles2) {
  assert(angles1.size() == angles2.size());
  double total = 0.0;
  for (int i=0; i < angles1.size(); ++i) {
    auto t1 = wrap_angle(angles1[i]);
    auto t2 = wrap_angle(angles2[i]);
    total += pow(t1 - t2, 2);
  }
  return sqrt(total);
}

inline double euclidean_distance(State s1, State s2) {
  auto base_distance = euclidean_distance(s1.base_pose, s2.base_pose);
  auto angle_distance = euclidean_distance(s1.link_angles, s2.link_angles);

  return base_distance + angle_distance;
}
