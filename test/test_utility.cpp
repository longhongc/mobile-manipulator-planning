/* test_utility.cpp
 *
 * Author: Chang-Hong Chen
 * Email: longhongc@gmail.com
 */

#include "utility.h"

#include <gtest/gtest.h>

TEST(Utility, wrap_angle) {
  double t1 = wrap_angle(3 * M_PI);
  EXPECT_DOUBLE_EQ(t1, M_PI);

  double t2 = wrap_angle(-3.5 * M_PI);
  EXPECT_DOUBLE_EQ(t2, -1.5 * M_PI);
}
