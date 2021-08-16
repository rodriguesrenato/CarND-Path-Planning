
#ifndef TRAJECTORY_H
#define TRAJECTORY_H
#include <string>
#include <vector>
#include "states.h"
using std::string;
using std::vector;

class Trajectory {
 public:
    Trajectory();
    vector<double> x{};
    vector<double> y{};
    double cost{0};
    double prev_speed{0};
    double prev_lane{0};
    double final_speed{0};
    double final_lane{0};
    double target_speed{0};
    State state{State::R};
};
#endif