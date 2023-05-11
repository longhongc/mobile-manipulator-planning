/* rrt_cspace.cpp
 *
 * Author: Chang-Hong Chen
 * Email: longhongc@gmail.com
 */

#include <algorithm>
#include <memory>
#include <iostream>
#include <fstream>
#include <limits>
#include <random>

#include "rrt_cspace.h"

Node::Node(State s, std::vector<State> wp, std::shared_ptr<Node> p, Pose ee_p):
  state{s},
  waypoints{wp},
  parent{p},
  ee_pose{ee_p} {
}

RRTCSpace::RRTCSpace(State start, Coord2D goal,
    std::shared_ptr<Problem> problem_ptr):
  start_state_{start},
  goal_coord_{goal}, 
  problem_ptr_{problem_ptr}{
  // if (!problem_ptr_->valid_coord(start_state_.base_pose)) {
    // throw std::invalid_argument("Start coordinate out of bound");
  // }

  // if (problem_ptr_->check_collision(start_coord_)) {
    // throw std::invalid_argument("Start coordinate in obstacle");
  // }

  auto ee_pose = problem_ptr_->robot->getEEPose(start_state_);
  std::shared_ptr<Node> start_node =
    std::make_shared<Node>(
        start_state_, std::vector<State>(), nullptr, ee_pose);
  nodes_.push_back(start_node);
}

std::vector<State> RRTCSpace::solve() {
  auto start_time = std::chrono::steady_clock::now();
  // Total duration of RRT
  std::chrono::seconds duration_s{0};

  std::chrono::milliseconds duration_ms{0};

  // Everytime duration passes the threshold
  // print out a time message
  std::chrono::seconds duration_threshold{1};

  while (duration_s < RRT_TIME_LIMIT) {
    auto curr_node = this->sample_node();
    if (curr_node == nullptr) {
      continue;
    }

    auto ee_coord = curr_node->ee_pose.position;

    if (euclidean_distance(ee_coord, goal_coord_) <= goal_tolerance_) {
      std::cout << "\033[32mFound solution\033[0m" << std::endl;
      search_time_ = duration_ms.count();
      return this->generate_path(curr_node);
    }

    duration_s = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::steady_clock::now() - start_time);
    duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start_time);

    if (duration_s > duration_threshold) {
      std::cout << "Search time: [" << duration_s.count() << "s]" << std::endl;
      duration_threshold++;
    }
  }
  std::cout << "\033[31mSolution not found in time limit\033[0m" << std::endl;
  return {};
}

std::shared_ptr<Node> RRTCSpace::find_nearest_node(Coord2D new_ee) {
  std::shared_ptr<Node> nearest_node_ptr;
  double min_distance = std::numeric_limits<double>::max();

  for (auto node_ptr : nodes_) {
    auto curr_ee = node_ptr->ee_pose.position;
    auto curr_distance = euclidean_distance(new_ee, curr_ee);

    if (curr_distance < min_distance) {
      min_distance = curr_distance;
      nearest_node_ptr = node_ptr;
    }
  }

  return nearest_node_ptr;
}

std::shared_ptr<Node> RRTCSpace::sample_node() {
  // Create uniform distribution
  static std::random_device rd;
  static std::mt19937 gen(rd());
  static std::uniform_real_distribution<double> uniform_sample_x(
    problem_ptr_->X_MIN, problem_ptr_->X_MAX);
  static std::uniform_real_distribution<double> uniform_sample_y(
    problem_ptr_->Y_MIN, problem_ptr_->Y_MAX);

  // Sample a random coordinate
  Coord2D random_coord = std::make_pair(
      uniform_sample_x(gen), uniform_sample_y(gen));

  // std::normal_distribution<double> gaussian_theta(0, M_PI / 2);

  auto nearest_node_ptr = this->find_nearest_node(random_coord);
  auto nearest_state = nearest_node_ptr->state;

  static std::uniform_real_distribution<double> uniform_sample_vdot(
    problem_ptr_->V_DOT_MIN, problem_ptr_->V_DOT_MAX);
  static std::uniform_real_distribution<double> uniform_sample_wdot(
    problem_ptr_->W_DOT_MIN, problem_ptr_->W_DOT_MAX);

  auto angle_input = uniform_sample_wdot(gen);
  StateInput random_input = 
    {uniform_sample_vdot(gen), uniform_sample_vdot(gen), 
     angle_input, -angle_input};

  // StateInput random_input = 
    // {uniform_sample_vdot(gen), uniform_sample_vdot(gen), 
     // uniform_sample_wdot(gen), uniform_sample_wdot(gen)}; 

  auto traj = problem_ptr_->robot->forward_simulation(
    nearest_state, random_input);

   if (traj.empty()) {
    return nullptr;
  }

  // Check if any waypoints in the trajectory is already in goal
  // and cut the trajectory to that waypoint
  int cut_index = -1;
  Pose new_ee_pose;
  for (int i=0; i < traj.size(); ++i) {
    new_ee_pose = problem_ptr_->robot->getEEPose(traj[i]);
    auto ee_coord = new_ee_pose.position;
    if (euclidean_distance(ee_coord, goal_coord_) <= goal_tolerance_) {
      cut_index = i + 1;
      break;
    }
  }

  if (cut_index > 0 &&
      cut_index < traj.size()) {
    traj.erase(traj.begin() + cut_index, traj.end());
  }

  // Return nullptr if this new sampled coordinate is invalid
  if (problem_ptr_->check_collision(traj)) {
    return nullptr;
  }

  auto new_state = traj.back();
  std::shared_ptr<Node> new_node =
    std::make_shared<Node>(
      new_state, traj, nearest_node_ptr, new_ee_pose);

  nodes_.push_back(new_node);

  return new_node;
}

std::vector<State> RRTCSpace::generate_path(std::shared_ptr<Node> node_ptr) {
  auto curr_node_ptr = node_ptr;
  std::vector<State> path;
  // Backtracking from current node
  while (curr_node_ptr != nullptr) {
    for (int i=curr_node_ptr->waypoints.size() - 1;
        i >= 0; --i) {
      path.push_back(curr_node_ptr->waypoints[i]);
    }
    curr_node_ptr = curr_node_ptr->parent;
  }

  std::reverse(path.begin(), path.end());
  path_ = path;
  return path;
}

void RRTCSpace::print_path() {
  std::string content;
  for (auto& state : path_) {
    print_state(state);
  }
}

void RRTCSpace::save_path_to_file(std::string file_name) {
  std::ofstream file{file_name};
  std::string content;
  for (auto& state : path_) {
    auto poses = problem_ptr_->robot->FK(state);

    std::string line;
    for (auto& pose : poses) {
      auto& [x, y] = pose.position;
      auto& t = pose.angle;
      line += std::to_string(x) + ", " +
        std::to_string(y) + ", " +
        std::to_string(t) + ", ";
    }

    line += std::to_string(state.time);
    line += "\n";
    content += line;
  }
  file << content;
  file.close();
}

void RRTCSpace::save_search_tree_to_file(std::string file_name) {
  return;
}

double RRTCSpace::search_time() {
  return search_time_;
}

double RRTCSpace::search_node() {
  return nodes_.size();
}

double RRTCSpace::path_time() {
  return path_.back().time;
}

