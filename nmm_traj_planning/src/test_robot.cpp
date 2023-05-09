#include <iostream>
#include <memory>

#include "utility.h"
#include "robot.h"

int main(int argc, char** argv) {

  State start_state{
    {0.0, 0.0, 0.0, 0.0, 0.0}, {0, 0, 0, 0}, 0};

  NonholonomicMM robot;

  auto start_ee = robot.getEEPose(start_state);
  print_pose(start_ee);

  StateInput state_input{0.5, 0.5, 0, 0};
  auto traj = robot.forward_simulation(
    start_state, state_input, 5, 1);

  std::cout << "Traj size: " <<  traj.size() << std::endl;

  for (auto& state : traj) {
    print_state(state);
  }

  return 0;
}

