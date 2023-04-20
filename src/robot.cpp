/* robot.cpp
 *
 * Author: Chang-Hong Chen
 * Email: longhongc@gmail.com
 */

#include <cmath>

#include "robot.h"

HolonomicMM::HolonomicMM() {}

EEPose HolonomicMM::FK(const State& state) {
  auto [x, y] = state.base_pose;
  auto t1 = state.link_angles.at(0);
  auto t2 = state.link_angles.at(1);

  auto l1 = link_lengths_.at(0);
  auto l2 = link_lengths_.at(1);

  double x_e = x + l1 * cos(t1) + l2 * cos(t1 + t2);
  double y_e = y + l1 * sin(t1) + l2 * sin(t1 + t2);
  double t_e = wrap_angle(t1 + t2);

  EEPose ee{{x_e, y_e}, t_e};
  return ee;
}


