/* robot.h
 *
 * Author: Chang-Hong Chen
 * Email: longhongc@gmail.com
 */

#pragma once

#include <array>
#include <vector>
#include <string>

#include <drake/systems/framework/leaf_system.h>
#include <drake/systems/framework/basic_vector.h>
#include <drake/systems/framework/context.h>
#include <drake/systems/framework/continuous_state.h>

#include "utility.h"

#define STATE_NUM 9
#define INPUT_NUM 4

struct State {
  State();
  State(std::vector<double> v, std::vector<double> d, double t);
  std::string to_string();
  std::vector<double> values; // x, y, t, t1, t2
  std::vector<double> dots; // v, tdot, t1dot, t2dot
  std::vector<double> full_states;
  double time;
};

void print_state(State& state);

// vdot, tddot, t1ddot, t2ddot
using StateInput = std::array<double, INPUT_NUM>;

class NonholonomicMM {
  public:
    NonholonomicMM();
    Pose getEEPose(const State& state);
    std::vector<Pose> FK(const State& state);
    bool checkSelfCollision(const State& state, int points = 5);

    std::vector<State> forward_simulation(
      State& s,
      StateInput& s_in,
      double epsilon = 5,
      double delta = 1,
      double boundary_time = 3);

    const double base_radius = 3.0;
    const std::vector<double> link_lengths{5.0, 5.0};
    const double link_offset = 2.0; // d
    double workspace_radius; 
};

class NMMSystem : public drake::systems::LeafSystem<double> {
  public:
    NMMSystem();

  private:
    void DoCalcTimeDerivatives(
        const drake::systems::Context<double>& context,
        drake::systems::ContinuousState<double>* derivatives) const;

    void CopyStateOut(
        const drake::systems::Context<double>& context,
        drake::systems::BasicVector<double>* output) const;

    bool t1_at_limit(double t1) const;

    const std::vector<double> max_state_velocity
      {3, 0.25 * M_PI, 0.25 * M_PI, 0.25 * M_PI};

    const std::vector<double> min_state_velocity
      {-3, -0.25 * M_PI, -0.25 * M_PI, -0.25 * M_PI};
};
