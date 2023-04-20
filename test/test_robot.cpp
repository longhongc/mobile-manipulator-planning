/* test_robot.cpp
 *
 * Author: Chang-Hong Chen
 * Email: longhongc@gmail.com
 */
#include <memory>

#include "robot.h"

#include <gtest/gtest.h>

class TestHolonomicMM : public ::testing::Test {
 public:
  TestHolonomicMM() {}

  void SetUp() {
    hmm_ptr_ = std::make_unique<HolonomicMM>();
  }

 protected:
  std::unique_ptr<HolonomicMM> hmm_ptr_;
};

TEST_F(TestHolonomicMM, FK) {
  State s1{{0, 0}, {0, 0}};
  auto ee1 = hmm_ptr_->FK(s1);
  EXPECT_DOUBLE_EQ(ee1.position.first, 10);
  EXPECT_DOUBLE_EQ(ee1.position.second, 0);
  EXPECT_DOUBLE_EQ(ee1.angle, 0);

  State s2{{0, 0}, {-M_PI / 2, M_PI / 2}};
  auto ee2 = hmm_ptr_->FK(s2);
  EXPECT_DOUBLE_EQ(ee2.position.first, 5);
  EXPECT_DOUBLE_EQ(ee2.position.second, -5);
  EXPECT_DOUBLE_EQ(ee2.angle, 0);

  State s3{{2, -10}, {0, M_PI / 2}};
  auto ee3 = hmm_ptr_->FK(s3);
  EXPECT_DOUBLE_EQ(ee3.position.first, 7);
  EXPECT_DOUBLE_EQ(ee3.position.second, -5);
  EXPECT_DOUBLE_EQ(ee3.angle, M_PI / 2);
}
