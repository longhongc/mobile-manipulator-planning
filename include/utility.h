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
using Radius = double;
using Circle = std::pair<Coord2D, Radius>;

struct Pose {
  Coord2D position;
  double angle;
};

struct State {
  Coord2D base_pose;
  std::vector<double> joint_angles;
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
  auto angle_distance = euclidean_distance(s1.joint_angles, s2.joint_angles);

  return base_distance + angle_distance;
}

inline std::vector<Coord2D> interpolate(Coord2D coord1, Coord2D coord2, int points = 5) {
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

inline std::vector<std::vector<double>> interpolate(
    std::vector<double> angles1, std::vector<double> angles2, int points = 5) {


  for (int i=0; i < 2; ++i) {
    angles1[i] = wrap_angle(angles1[i]);
    angles2[i] = wrap_angle(angles2[i]);
  }

  std::vector<double> angle_steps;
  for (int i=0; i < 2; ++i) {
    double step = (angles2[i] - angles1[i]) / (points - 1);
    angle_steps.push_back(step);
  }

  std::vector<std::vector<double>> intermediate_joints;
  for (int i=0; i < points ; ++i) {
    std::vector<double> joint;
    for (int j=0; j < 2; ++j) {
      joint.push_back(
          wrap_angle(angles1[j] + angle_steps[j] * i));
    }
    intermediate_joints.push_back(joint);
  }

  return intermediate_joints;
}

inline std::vector<State> interpolate(State s1, State s2, int points = 5) {
  auto intermediate_coords = interpolate(s1.base_pose, s2.base_pose, points);
  auto intermediate_joints = interpolate(s1.joint_angles, s2.joint_angles, points);

  std::vector<State> intermediate_states;
  for (int i=0; i < points ; ++i) {
    auto coord = intermediate_coords[i];
    auto joints = intermediate_joints[i];
    intermediate_states.push_back({coord, joints});
  }

  return intermediate_states;
}
