/* utility.h
 *
 * Author: Chang-Hong Chen
 * Email: longhongc@gmail.com
 */

#pragma once

#include <cmath>
#include <cassert>
#include <utility>
#include <vector>

using Coord2D = std::pair<double, double>;
using Radius = double;
using Circle = std::pair<Coord2D, Radius>;

struct Pose {
  Coord2D position;
  double angle;
};

double wrap_angle(double angle);

double euclidean_distance(Coord2D coord1, Coord2D coord2);
double euclidean_distance(std::vector<double> v1, std::vector<double> v2);

std::vector<Coord2D> interpolate(Coord2D coord1, Coord2D coord2, int points = 5);

void print_pose(Pose p);
