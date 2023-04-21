/* problem.h
 *
 * Author: Chang-Hong Chen
 * Email: longhongc@gmail.com
 */

#pragma once

#include <memory>

#include "utility.h"
#include "robot.h"

class Problem {
  public:
    Problem();

    bool valid_coord(Coord2D coord);

    const double X_MAX = 50;
    const double X_MIN = -50;
    const double Y_MAX = 50;
    const double Y_MIN = -50;

    std::unique_ptr<HolonomicMM> robot = std::make_unique<HolonomicMM>();
};
