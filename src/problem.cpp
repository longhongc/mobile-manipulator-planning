/* problem.cpp
 *
 * Author: Chang-Hong Chen
 * Email: longhongc@gmail.com
 */

#include "problem.h"

Problem::Problem() {}

bool Problem::valid_coord(Coord2D coord) {
  auto& [x, y] = coord;
  if (x < X_MIN || x > X_MAX ||
      y < Y_MIN || y > Y_MAX) {
    return false;
  }

  return true;
}
