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
    Problem(std::string obstacles_file);

    bool valid_coord(Coord2D coord);
    bool check_collision(Coord2D coord, double tolerance = 0.0); 
    bool check_collision(Coord2D coord1, Coord2D coord2, int points = 5);
    bool check_collision(State state1, State state2, int points = 5);

    const double X_MAX = 50;
    const double X_MIN = -50;
    const double Y_MAX = 50;
    const double Y_MIN = -50;

    std::unique_ptr<HolonomicMM> robot = std::make_unique<HolonomicMM>();

  private:
    void load_workspace(std::string obstacles_file);

    std::vector<Circle> circular_obstacles_;
};
