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
    bool checkSelfCollision(const State& state, int points = 5);

    const double base_radius = 3.0;
    const std::vector<double> link_lengths{5.0, 5.0};
    double workspace_radius; 
};
