/* test_utility.cpp
 *
 * Author: Chang-Hong Chen
 * Email: longhongc@gmail.com
 */

#include "utility.h"

#include <gtest/gtest.h>

#define epsilon 1e-3

TEST(Utility, wrap_angle) {
  double t1 = wrap_angle(3 * M_PI);
  EXPECT_DOUBLE_EQ(t1, M_PI);

  double t2 = wrap_angle(-3.5 * M_PI);
  EXPECT_DOUBLE_EQ(t2, 0.5 * M_PI);
}

TEST(Utility, euclidean_distance) {
  Coord2D c1{2, 5};
  Coord2D c2{-2, 0};
  auto ans1 = euclidean_distance(c1, c2);
  EXPECT_DOUBLE_EQ(ans1, sqrt(41));

  std::vector<double> j1{3, -7};
  std::vector<double> j2{-5, 3};
  auto ans2 = euclidean_distance(j1, j2);
  EXPECT_NEAR(ans2, sqrt(9.53371), epsilon);

  State s1{c1, j1};
  State s2{c2, j2};
  auto ans3 = euclidean_distance(s1, s2);
  EXPECT_DOUBLE_EQ(ans3, ans1 + ans2);
}

TEST(Utility, interpolate) {
  Coord2D c1{2, 5};
  Coord2D c2{-2, 0};
  auto c_vec = interpolate(c1, c2, 3);
  EXPECT_EQ(c_vec.size(), 3);
  EXPECT_DOUBLE_EQ(c_vec[1].first, 0);
  EXPECT_DOUBLE_EQ(c_vec[1].second, 2.5);

  std::vector<double> j1{0, 2};
  std::vector<double> j2{3, -3};
  auto j_vec = interpolate(j1, j2, 3);
  EXPECT_EQ(j_vec.size(), 3);
  EXPECT_NEAR(j_vec[1][0], 1.5, epsilon);
  EXPECT_NEAR(j_vec[1][1], 2.6416, epsilon);

  State s1{c1, j1};
  State s2{c2, j2};
  auto s_vec = interpolate(s1, s2, 3);
  EXPECT_EQ(s_vec.size(), 3);
  EXPECT_DOUBLE_EQ(s_vec[1].base_pose.first, 0);
  EXPECT_DOUBLE_EQ(s_vec[1].base_pose.second, 2.5);
  EXPECT_NEAR(s_vec[1].joint_angles[0], 1.5, epsilon);
  EXPECT_NEAR(s_vec[1].joint_angles[1], 2.6416, epsilon);
}
