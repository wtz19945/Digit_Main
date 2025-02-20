#ifndef PAHT_FINDER_MAIN_h
#define PAHT_FINDER_MAIN_h
/*
Path Find Block
Input : Digit state   (current CoM info)
Output: A Collision-free path computed by Astar
*/

#pragma once
// C++ Standard
#include <iostream>
#include <vector>
#include <chrono>
#include <vector>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <set>
#include <cassert>

// Eigen pack
#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Dense>
#include <Eigen/Core>
#include <filesystem>

// ROS pack
#include "ros/ros.h"
#include <ros/package.h>
#include <std_msgs/Int8.h>
#include "Digit_Ros/digit_state.h"
#include "Digit_Ros/mpc_info.h"
#include "input_listener.hpp"
#include "cpptoml/include/cpptoml.h"

// casadi pack
#include <casadi/casadi.hpp>

// custom pack
#include "mpc_solver.hpp"

class Astar_Planner {
public:
  Astar_Planner();
  void Plan_Path();
  void Update_Map(const std::vector<std::vector<int>>& new_grid) {
    grid_ = new_grid; 
    rows_ = grid_.size();
    cols_ = grid_[0].size();};

  void Update_Start(const std::pair<int, int>& new_start) {
    assert(new_start.first >= 0 && "invalid state position");
    assert(new_start.first < rows_ && "invalid state position");
    assert(new_start.second >= 0 && "invalid state position");
    assert(new_start.second < cols_ && "invalid state position");

    start_ = new_start;};
  void Update_Goal(const std::pair<int, int>& new_goal) {
    assert(new_goal.first >= 0 && "invalid goal position");
    assert(new_goal.first < rows_ && "invalid goal position");
    assert(new_goal.second >= 0 && "invalid goal position");
    assert(new_goal.second < cols_ && "invalid goal position");

    goal_ = new_goal;};
  void savePathToFile();

private:
  std::vector<std::vector<int>> grid_;
  std::pair<int, int> start_;
  std::pair<int, int> goal_;
  std::vector<std::pair<int, int>> path_;
  int rows_;
  int cols_;
  const std::vector<std::pair<int, int>> DIRECTIONS_4_ = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};
  const std::vector<std::pair<int, int>> DIRECTIONS_8_ = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}, {1, 1}, {1, -1}, {-1, -1}, {-1, 1}};
  std::string path_file_name_;
  std::string grid_file_name_;
};

// Node structure for A*
struct Node {
    int x, y;
    double g = 1e8, h, f = 1e8;
    
    bool operator<(const Node& other) const {
        return std::tie(x, y) < std::tie(other.x, other.y);
    }

    bool operator==(const Node& other) const { return x == other.x && y == other.y; }


};

std::ostream& operator<<(std::ostream& os, const Node& s) {
    os << "Nodes { x: " << s.x << ", y: " << s.y << ", f: \"" << s.f << "\" }";
    return os;
}

// Hash function for unordered_set and unordered_map
struct NodeHash {
    size_t operator()(const Node& n) const {
        return std::hash<int>()(n.x) ^ std::hash<int>()(n.y);
    }
};

// Heuristic function (Euclidean Distance)
double heuristic(int x1, int y1, int x2, int y2) {
    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}


#endif //MPC_MAIN_H