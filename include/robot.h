/* robot.h
 *
 * Author: Chang-Hong Chen
 * Email: longhongc@gmail.com
 */

#pragma once

#include <vector>

#include "utility.h"

struct State {
  Coord2D base_pose;
  std::vector<double> link_angles;
};

struct EEPose {
  Coord2D position;
  double angle;
};

class HolonomicMM {
  public:
    HolonomicMM();
    EEPose FK(const State& state);

  private:
    const double base_radius_ = 5.0;
    const std::vector<double> link_lengths_{5.0, 5.0};
};
