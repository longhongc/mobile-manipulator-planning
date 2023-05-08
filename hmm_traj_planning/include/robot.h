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

#define STATE_NUM 8
#define INPUT_NUM 4

struct State {
  State();
  State(std::vector<double> v, std::vector<double> d, double t);
  std::string to_string();
  std::vector<double> values;
  std::vector<double> dots;
  std::vector<double> full_states;
  double time;
};

void print_state(State& state);

using StateInput = std::array<double, INPUT_NUM>;

class HolonomicMM {
  public:
    HolonomicMM();
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
    double workspace_radius; 
};

class HMMSystem : public drake::systems::LeafSystem<double> {
  public:
    HMMSystem();

  private:
    void DoCalcTimeDerivatives(
        const drake::systems::Context<double>& context,
        drake::systems::ContinuousState<double>* derivatives) const;

    void CopyStateOut(
        const drake::systems::Context<double>& context,
        drake::systems::BasicVector<double>* output) const;

    const std::vector<double> max_state_velocity
      {3, 3, 0.25 * M_PI, 0.25 * M_PI};

    const std::vector<double> min_state_velocity
      {-3, -3, -0.25 * M_PI, -0.25 * M_PI};
};
