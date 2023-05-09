/* robot.cpp
 *
 * Author: Chang-Hong Chen
 * Email: longhongc@gmail.com
 */

#include <cmath>
#include <numeric>
#include <iostream>
#include <string>

#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/primitives/vector_log_sink.h>
#include <drake/systems/primitives/constant_vector_source.h>

#include <Eigen/Dense>

#include "robot.h"

State::State() {};
State::State(std::vector<double> v, std::vector<double> d, double t) :
  values{v}, 
  dots{d}, 
  time{t} {
  full_states.insert(full_states.end(), v.begin(), v.end());
  full_states.insert(full_states.end(), d.begin(), d.end());
}

std::string State::to_string() {
  std::string state_str;

  for (auto& v : full_states) {
    state_str += std::to_string(v);
    state_str += ", ";
  }
  state_str += std::to_string(time);

  return state_str;
}

void print_state(State& state) {
  std::cout << state.to_string() << std::endl;
}

NonholonomicMM::NonholonomicMM() {
  workspace_radius = std::max(base_radius, 
      std::accumulate(link_lengths.begin(), link_lengths.end(), 0.0)) + link_offset;
}

Pose NonholonomicMM::getEEPose(const State& state) {
  return this->FK(state).back();
}

std::vector<Pose> NonholonomicMM::FK(const State& state) {
  auto x = state.values.at(0);
  auto y = state.values.at(1);
  auto t = state.values.at(2);
  auto t1 = state.values.at(3);
  auto t2 = state.values.at(4);

  double d = link_offset;
  auto l1 = link_lengths.at(0);
  auto l2 = link_lengths.at(1);

  std::vector<Pose> p;
  p.push_back({{x, y}, t});

  double x1 = x + d * cos(t) + l1 * cos(t + t1);
  double y1 = y + d * sin(t) + l1 * sin(t + t1);
  p.push_back({{x1, y1}, wrap_angle(t + t1)});

  double xe = x1 + l2 * cos(t + t1 + t2);
  double ye = y1 + l2 * sin(t + t1 + t2);
  double te = wrap_angle(t + t1 + t2);

  Pose ee{{xe, ye}, te};
  p.push_back(ee);

  return p;
}

bool NonholonomicMM::checkSelfCollision(const State& state, int points) {
  auto poses = this->FK(state);
  auto base = poses[0];
  auto j1 = poses[1];
  auto j2 = poses[2];

  auto intermediate_coords = interpolate(j1.position, j2.position, points);

  for (int i=0; i < points ; ++i) {
    auto coord = intermediate_coords[i];
    auto dist = euclidean_distance(base.position, coord);
    if (dist < base_radius) {
      return true;
    }
  }

  return false;
}

std::vector<State> NonholonomicMM::forward_simulation(
    State& init_state,
    StateInput& state_input,
    double epsilon,
    double delta,
    double boundary_time) {

  drake::systems::DiagramBuilder<double> builder;
  auto system = builder.AddSystem(std::make_unique<NMMSystem>());

  Eigen::VectorXd acc(state_input.size());
  std::copy(state_input.begin(), state_input.end(), acc.data());

  auto system_input = builder.AddSystem(
      std::make_unique<drake::systems::ConstantVectorSource<double>>(acc));

  builder.Connect(system_input->get_output_port(), system->get_input_port(0));

  auto logger = LogVectorOutput(system->get_output_port(0), &builder);
  auto diagram = builder.Build();

  auto context = diagram->CreateDefaultContext();

  auto full_states = init_state.full_states;
  Eigen::VectorXd v(full_states.size());
  std::copy(full_states.begin(), full_states.end(), v.data());

  context->SetContinuousState(v);

  drake::systems::Simulator<double> simulator(*diagram, std::move(context));

  // Start forward simulation
  simulator.AdvanceTo(boundary_time);
  auto log = logger->FindLog(simulator.get_context());

  std::vector<State> trajectory;

  double ee_dist = 0.0;
  double total_ee_travel = 0.0;
  double prev_time = log.sample_times()[0];
  auto prev_ee = this->getEEPose(init_state);

  for (int i = 1; i < log.sample_times().size(); ++i) {
    auto x = log.data()(i * STATE_NUM + 0);
    auto y = log.data()(i * STATE_NUM + 1);
    auto t = wrap_angle(log.data()(i * STATE_NUM + 2));
    auto t1 = wrap_angle(log.data()(i * STATE_NUM + 3));
    auto t2 = wrap_angle(log.data()(i * STATE_NUM + 4));

    auto v = log.data()(i * STATE_NUM + 5);
    auto vt = log.data()(i * STATE_NUM + 6);
    auto vt1 = log.data()(i * STATE_NUM + 7);
    auto vt2 = log.data()(i * STATE_NUM + 8);
    double curr_time = log.sample_times()[i];
    double state_time = curr_time + init_state.time;

    State curr_state({x, y, t, t1, t2}, {v, vt, vt1, vt2}, state_time);
    auto curr_ee = this->getEEPose(curr_state);
    ee_dist += euclidean_distance(prev_ee.position, curr_ee.position);

    prev_ee = curr_ee;

    if (ee_dist < delta &&
        i != (log.sample_times().size() - 1)) {
      continue;
    }

    trajectory.push_back(curr_state);

    total_ee_travel += ee_dist;

    if (total_ee_travel > epsilon) {
      break;
    }
    ee_dist = 0.0;
  }

  return trajectory;
}

NMMSystem::NMMSystem() {
  DeclareContinuousState(STATE_NUM);
  DeclareVectorInputPort("acc", drake::systems::BasicVector<double>(INPUT_NUM));
  DeclareVectorOutputPort("state", drake::systems::BasicVector<double>(STATE_NUM),
                          &NMMSystem::CopyStateOut);
}

void NMMSystem::DoCalcTimeDerivatives(
    const drake::systems::Context<double>& context,
    drake::systems::ContinuousState<double>* derivatives) const {

  auto acc = get_input_port(0).Eval(context);

  auto t = context.get_continuous_state()[2];
  auto t1 = context.get_continuous_state()[3];
  auto v = context.get_continuous_state()[5];
  auto vt = context.get_continuous_state()[6];
  auto vt1 = context.get_continuous_state()[7];
  auto vt2 = context.get_continuous_state()[8];

  (*derivatives)[0] = v * cos(t);
  (*derivatives)[1] = v * sin(t);
  (*derivatives)[2] = vt;

  if (this->t1_at_limit(t1)) {
    (*derivatives)[3] = 0;
    acc[2] = 0;
  } else {
    (*derivatives)[3] = vt1;
  }

  (*derivatives)[4] = vt2;

  for (int i=0; i < 4; ++i) {
    int vel_state_index = i + 5;
    auto v_i = context.get_continuous_state()[vel_state_index];

    auto v_max = max_state_velocity[i];
    auto v_min = min_state_velocity[i];
    auto a = acc[i];

    if ((v_i > v_max && a > 0) ||
        (v_i < v_min && a < 0)) {
      (*derivatives)[vel_state_index] = 0;
    } else {
      (*derivatives)[vel_state_index] = a;
    }
  }
}

void NMMSystem::CopyStateOut(const drake::systems::Context<double>& context,
                                 drake::systems::BasicVector<double>* output) const {
  for (int i = 0; i < context.get_continuous_state().size(); ++i) {
    (*output)[i] = context.get_continuous_state()[i];
  }
}


bool NMMSystem::t1_at_limit(double t1) const {
  auto wrapped = wrap_angle(t1);
  if (wrapped <= (M_PI / 3.0) || wrapped >= (5.0 * M_PI / 3.0)) {
    return false;
  }
   
  return true;
}
