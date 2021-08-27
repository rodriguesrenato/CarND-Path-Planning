
#ifndef TRAJECTORY_H
#define TRAJECTORY_H
#include <limits>
#include <string>
#include <vector>

#include "states.h"
using std::string;
using std::vector;

class Trajectory {
 public:
  Trajectory();
  vector<double> x{};          // Vector of x positions
  vector<double> y{};          // Vector of y positions
  double final_s{0};           // Final s positions
  double final_d{0};           // Final d positions
  double final_speed{0};       // Final speed positions
  double final_speed_prev{0};  // Final speed positions from previous trajectory
  int projected_lane{0};       // Projected lane
  double projected_speed{0};   // Projected speed
  double target_speed{0};      // Target speed for this trajectory
  int target_lane{0};          // Target lane
  float cost{std::numeric_limits<float>::max() - 1.0};  // Total cost
  vector<float> costs{};                                // Vector of costs
  State state{State::R};                                // Trajectory State
  State prev_state{State::R};                           // Previous State
};
#endif