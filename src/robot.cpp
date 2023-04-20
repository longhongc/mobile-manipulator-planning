/* robot.cpp
 *
 * Author: Chang-Hong Chen
 * Email: longhongc@gmail.com
 */

#include <cmath>

#include "robot.h"

HolonomicMM::HolonomicMM() {}

Pose HolonomicMM::getEEPose(const State& state) {
  return this->FK(state).back();
}

std::vector<Pose> HolonomicMM::FK(const State& state) {
  auto [x, y] = state.base_pose;
  auto t1 = state.link_angles.at(0);
  auto t2 = state.link_angles.at(1);

  auto l1 = link_lengths_.at(0);
  auto l2 = link_lengths_.at(1);

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


