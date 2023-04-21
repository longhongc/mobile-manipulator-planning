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

Node::Node(State s, std::shared_ptr<Node> p):
  state{s}, 
  parent{p} {
}

RRTCSpace::RRTCSpace(State start, Coord2D goal,
    std::shared_ptr<Problem> problem_ptr):
  start_state_{start},
  goal_coord_{goal}, 
  problem_ptr_{problem_ptr}{
  if (!problem_ptr_->valid_coord(start_state_.base_pose)) {
    throw std::invalid_argument("Start coordinate out of bound");
  }

  // if (problem_ptr_->check_collision(start_coord_)) {
    // throw std::invalid_argument("Start coordinate in obstacle");
  // }

  std::shared_ptr<Node> start_node =
    std::make_shared<Node>(start_state_, nullptr);
  nodes_.push_back(start_node);
}

std::vector<State> RRTCSpace::solve() {
  auto start_time = std::chrono::steady_clock::now();
  // Total duration of RRT
  std::chrono::seconds duration_s{0};

  // Everytime duration passes the threshold
  // print out a time message
  std::chrono::seconds duration_threshold{1};

  while (duration_s < RRT_TIME_LIMIT) {
    auto curr_node = this->sample_node();
    if (curr_node == nullptr) {
      continue;
    }

    auto ee_coord = problem_ptr_->robot->getEEPose(curr_node->state).position;

    if (euclidean_distance(ee_coord, goal_coord_) <= goal_tolerance_) {
      std::cout << "\033[32mFound solution\033[0m" << std::endl;
      return this->generate_path(curr_node);
    }

    duration_s = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::steady_clock::now() - start_time);

    if (duration_s > duration_threshold) {
      std::cout << "Search time: [" << duration_s.count() << "s]" << std::endl;
      duration_threshold++;
    }
  }
  std::cout << "\033[31mSolution not found in time limit\033[0m" << std::endl;
  return {};
}

std::shared_ptr<Node> RRTCSpace::find_nearest_node(State state) {
  std::shared_ptr<Node> nearest_node_ptr;
  double min_distance = std::numeric_limits<double>::max();

  for (auto node_ptr : nodes_) {
    auto curr_distance = euclidean_distance(node_ptr->state, state);

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
  static std::uniform_real_distribution<double> uniform_sample_t(
      0, 2 * M_PI);

  std::vector<double> random_joints;
  for (int i=0; i < 2; ++i) {
    random_joints.push_back(uniform_sample_t(gen));
  }

  State random_state{random_coord, random_joints};

  auto nearest_node_ptr = this->find_nearest_node(random_state);
  auto nearest_state = nearest_node_ptr->state;
  auto curr_distance = euclidean_distance(nearest_state, random_state);

  // If the distance is too long,
  // cut it into epsilon length
  if (curr_distance > rrt_epsilon_) {
    auto& [x1, y1] = nearest_state.base_pose;
    auto& [x2, y2] = random_state.base_pose;
    double cut_ratio = rrt_epsilon_ / curr_distance;
    double new_x = x1 + (x2 - x1) * cut_ratio;
    double new_y = y1 + (y2 - y1) * cut_ratio;
    random_coord = std::make_pair(new_x, new_y);

    for (int i=0; i < random_joints.size(); ++i) {
      auto t_near = wrap_angle(nearest_state.joint_angles[i]);
      auto t_random = wrap_angle(random_joints[i]);
      random_joints[i] = 
        wrap_angle(t_near + (t_random - t_near) * cut_ratio);
    }
  }

  random_state.base_pose = random_coord;
  random_state.joint_angles = random_joints;

  // Return nullptr if this new sampled coordinate is invalid
  if (problem_ptr_->check_collision(nearest_state, random_state)) {
    return nullptr;
  }

  std::shared_ptr<Node> new_node =
    std::make_shared<Node>(random_state, nearest_node_ptr);

  nodes_.push_back(new_node);
  // edges_.push_back(std::make_pair(nearest_coord, random_coord));

  return new_node;
}

std::vector<State> RRTCSpace::generate_path(std::shared_ptr<Node> node_ptr) {
  auto curr_node_ptr = node_ptr;
  std::vector<State> path;
  // Backtracking from current node
  while (curr_node_ptr != nullptr) {
    path.push_back(curr_node_ptr->state);
    curr_node_ptr = curr_node_ptr->parent;
  }

  std::reverse(path.begin(), path.end());
  path_ = path;
  return path;
}

void RRTCSpace::print_path() {
  std::string content;
  for (auto& state : path_) {
    auto& [x, y] = state.base_pose;
    std::string line =
      "Base: (" + std::to_string(x) + ", " +
      std::to_string(y) + "), ";

    line += "Joints: (";
    for (auto& joint : state.joint_angles) {
      line += std::to_string(joint) + ", ";
    }

    line += ")\n";
    content += line;
  }
  std::cout << content << std::endl;
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

    line.pop_back();
    line.pop_back();
    line += "\n";
    content += line;
  }
  file << content;
  file.close();
}

void RRTCSpace::save_search_tree_to_file(std::string file_name) {
  return;
}


