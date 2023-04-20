/* utility.h
 *
 * Author: Chang-Hong Chen
 * Email: longhongc@gmail.com
 */

#pragma once

#include <cmath>
#include <utility>

using Coord2D = std::pair<double, double>;

inline double wrap_angle(double angle) {
  return fmod(angle, (2 * M_PI));
}
