/* rrt_cspace.h
 *
 * Author: Chang-Hong Chen
 * Email: longhongc@gmail.com
 */

#pragma once

#include <chrono>
#include <memory>

#include "utility.h"
#include "problem.h"

struct Node {
  Node(State s, std::shared_ptr<Node> p);
  State state{{0.0, 0.0}, {0.0, 0.0}};
  std::shared_ptr<Node> parent;
};

class RRTCSpace {
  public: 
    RRTCSpace(State start, Coord2D goal, 
        std::shared_ptr<Problem> problem_ptr);

    std::vector<State> solve();
    void print_path();
    void save_path_to_file(std::string file_name);
    void save_search_tree_to_file(std::string file_name);

    const std::chrono::seconds RRT_TIME_LIMIT{5};
    
  private:
    std::shared_ptr<Node> find_nearest_node(State state);
    std::shared_ptr<Node> sample_node();
    std::vector<State> generate_path(std::shared_ptr<Node> node_ptr);

    std::shared_ptr<Problem> problem_ptr_;

    State start_state_{{0.0, 0.0}, {0.0, 0.0}};
    Coord2D goal_coord_{0.0, 0.0};

    const double goal_tolerance_ = 5;
    const double rrt_epsilon_ = 5.0;

    std::vector<std::shared_ptr<Node>> nodes_;

    std::vector<State> path_;
};
