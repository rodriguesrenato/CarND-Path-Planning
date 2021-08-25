
#ifndef TRAJECTORY_H
#define TRAJECTORY_H
#include <string>
#include <vector>
#include <limits> 
#include "states.h"
using std::string;
using std::vector;

class Trajectory {
 public:
    Trajectory();
    vector<double> x{};
    vector<double> y{};
    float cost{std::numeric_limits<float>::max() -1.0};
    double prev_speed{0};
    double prev_lane{0};
    double prev_d{0};
    double final_speed{0};
    double final_lane{0};
    double final_s{0};
    double final_d{0};
    double projected_lane{0};
    double lane_speed{0};
    double target_speed{0};
    int target_lane{0};
    vector<float> cost_debug{};
    string debug_msg{};
    State state{State::R};
    State prev_state{State::R};
};
#endif