#ifndef VEHICLE_H
#define VEHICLE_H

#include <map>
#include <string>
#include <vector>

#include "json.hpp"
#include "states.h"
#include "trajectory.h"

#define MPH_TO_MS 2.239

using nlohmann::json;
using std::map;
using std::string;
using std::vector;

class Vehicle {
 public:
  Vehicle(){};
  Vehicle(double road_speed_limit_mph, double speed_buffer_mph, int num_lanes,
          double time_step, double lane_width, double trajectory_s_projection,
          double safe_s_dist_ahead, double safe_s_dist_behind,
          int trajectory_buffer_size, double vehicle_length,
          double vehicle_width);
  void UpdateLocalizationData(double x, double y, double s, double d,
                              double yaw, double speed_mph);
  void UpdatePreviousPath(nlohmann::basic_json<> previous_path_x,
                          nlohmann::basic_json<> previous_path_y,
                          double end_path_s, double end_path_d);
  void UpdateSensorFusion(nlohmann::basic_json<> sensor_fusion);
  void SetChosenTrajectory(Trajectory &trajectory);
  vector<State> GetSuccessorStates();
  Trajectory GenerateTrajectory(State state, vector<double> &map_waypoints_s,
                                vector<double> &map_waypoints_x,
                                vector<double> &map_waypoints_y);
  Trajectory BuildTrajectory(State state, int target_lane,
                             int trajectory_buffer_size,
                             double trajectory_s_projection,
                             vector<double> &map_waypoints_s,
                             vector<double> &map_waypoints_x,
                             vector<double> &map_waypoints_y);
  void CalculateCost(Trajectory &trajectory);
  float SpeedCost(Trajectory &trajectory);
  float LaneChangeCost(Trajectory &trajectory);
  float CollisionCost(Trajectory &trajectory);
  bool CheckCollision(double init_s, double final_s, int init_lane,
                      int final_lane, int buffer_size);
  vector<vector<double>> PredictSensorFusion(int buffer_size);
  int CalculateLaneIndex(double d);
  int CalculateBestLane(State state);
  double CalculateLaneSpeed(int lane);
  vector<int> LanesToCheckSpeed(double d);
  vector<double> GetVehicleAhead(int lane);
  vector<double> GetVehicleBehind(int lane);
  void PrintStatistics(vector<Trajectory> t, bool only_state_change);

 private:
  // Measurements attributes given by the simulator
  double x_{0};
  double y_{0};
  double s_{0};
  double d_{0};
  double yaw_{0};
  double speed_{0};
  int lane_{0};
  nlohmann::basic_json<> sensor_fusion_{};
  nlohmann::basic_json<> previous_path_x_{};
  nlohmann::basic_json<> previous_path_y_{};
  double end_path_s_{0};
  double end_path_d_{0};

  // Attributes that are used between update steps
  State state_{State::R};
  State state_prev_{State::R};
  double target_speed_{0};
  double projected_speed_{0};
  double trajectory_final_speed_{0};
  int target_lane_{0};
  int projected_lane_{0};
  double trajectory_cost{0};

  // Pre calculated buffers
  map<int, vector<vector<double>>> predictions_{};
  map<int, double> lane_speed_{};

  // Cost weights
  vector<float> cost_weights_{1.0, 1.1, 1000.0};

  // Vehicle and trajectory configurations
  double vehicle_length_{5.0};
  double vehicle_width_{3.0};
  double safe_s_dist_ahead_{10.0};
  double safe_s_dist_behind_{2.0};
  double trajectory_s_projection_{30.0};
  int trajectory_buffer_size_{50};

  // Road and simulator configurations
  double speed_limit_{49.5};
  double speed_buffer_{0.5};
  int num_lanes_{3};
  double time_step_{0.02};
  double lane_width_{4.0};
};

#endif