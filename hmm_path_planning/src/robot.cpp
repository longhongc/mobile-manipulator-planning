/* robot.cpp
 *
 * Author: Chang-Hong Chen
 * Email: longhongc@gmail.com
 */

#include <cmath>
#include <numeric>

#include "robot.h"

HolonomicMM::HolonomicMM() {
  workspace_radius = std::max(base_radius, 
      std::accumulate(link_lengths.begin(), link_lengths.end(), 0.0));
}

Pose HolonomicMM::getEEPose(const State& state) {
  return this->FK(state).back();
}

std::vector<Pose> HolonomicMM::FK(const State& state) {
  auto [x, y] = state.base_pose;
  auto t1 = state.joint_angles.at(0);
  auto t2 = state.joint_angles.at(1);

  auto l1 = link_lengths.at(0);
  auto l2 = link_lengths.at(1);

  std::vector<Pose> p;
  p.push_back({{x, y}, 0});

  double x1 = x + l1 * cos(t1);
  double y1 = y + l1 * sin(t1);
  p.push_back({{x1, y1}, t1});

  double xe = x + l1 * cos(t1) + l2 * cos(t1 + t2);
  double ye = y + l1 * sin(t1) + l2 * sin(t1 + t2);
  double te = wrap_angle(t1 + t2);

  Pose ee{{xe, ye}, te};
  p.push_back(ee);

  return p;
}

bool HolonomicMM::checkSelfCollision(const State& state, int points) {
  auto poses = this->FK(state);
  auto base = poses[0];
  auto j1 = poses[1];
  auto j2 = poses[2];

  auto intermediate_coords = interpolate(j1.position, j2.position, points);

  for (int i=0; i < points ; ++i) {
    auto coord = intermediate_coords[i];
    auto dist = euclidean_distance(base.position, coord);
    if (dist < base_radius) {
      return true;
    }
  }

  return false;
}

