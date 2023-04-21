/* robot.h
 *
 * Author: Chang-Hong Chen
 * Email: longhongc@gmail.com
 */

#pragma once

#include <vector>

#include "utility.h"

class HolonomicMM {
  public:
    HolonomicMM();
    Pose getEEPose(const State& state);
    std::vector<Pose> FK(const State& state);

  private:
    const double base_radius_ = 3.0;
    const std::vector<double> link_lengths_{5.0, 5.0};
};
