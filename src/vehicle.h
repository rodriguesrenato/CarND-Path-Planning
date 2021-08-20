#ifndef VEHICLE_H
#define VEHICLE_H

#include <map>
#include <string>
#include <vector>

#include "states.h"
#include "trajectory.h"
// #include "helpers.h"
#include "json.hpp"

#define MPH_TO_MS 2.239

using nlohmann::json;

using std::map;
using std::string;
using std::vector;

class Vehicle {
 public:
  Vehicle(){};
  Vehicle(double road_speed_limit, double acc_limit, int num_lanes,
          double time_step, double lane_width);
  void SetState(State state);
  void SetLocalizationData(double x, double y, double s, double d, double yaw,
                           double speed_mph);
  void SetPrevPath(nlohmann::basic_json<> previous_path_x,
                   nlohmann::basic_json<> previous_path_y, double end_path_s,
                   double end_path_d);
  void SetSensorFusion(nlohmann::basic_json<> sensor_fusion);
  void SetChosenTrajectory(Trajectory &trajectory);
  int CalculateLaneIndex(double d);
  vector<State> GetSuccessorStates();
  Trajectory GenerateTrajectory(State state, vector<double> &map_waypoints_s,
                                vector<double> &map_waypoints_x,
                                vector<double> &map_waypoints_y);

  Trajectory GenerateTrajectoryKL(double target_traj_len,
                                  vector<double> &map_waypoints_s,
                                  vector<double> &map_waypoints_x,
                                  vector<double> &map_waypoints_y);
  Trajectory GenerateTrajectoryLC(State state, double target_traj_len,
                                  vector<double> &map_waypoints_s,
                                  vector<double> &map_waypoints_x,
                                  vector<double> &map_waypoints_y);
  Trajectory GenerateTrajectoryPLC(State state, double target_traj_len,
                                   vector<double> &map_waypoints_s,
                                   vector<double> &map_waypoints_x,
                                   vector<double> &map_waypoints_y);

  Trajectory GenerateTrajectoryKL2(double target_traj_len,
                                   vector<double> &map_waypoints_s,
                                   vector<double> &map_waypoints_x,
                                   vector<double> &map_waypoints_y);
  Trajectory GenerateTrajectoryLC2(State state, double target_traj_len,
                                   vector<double> &map_waypoints_s,
                                   vector<double> &map_waypoints_x,
                                   vector<double> &map_waypoints_y);
  Trajectory GenerateTrajectoryPLC2(State state, double target_traj_len,
                                    vector<double> &map_waypoints_s,
                                    vector<double> &map_waypoints_x,
                                    vector<double> &map_waypoints_y);

  Trajectory BuildTrajectory(State state, int target_lane, double target_speed,
                             int traj_size, double traj_s_len,
                             vector<double> &map_waypoints_s,
                             vector<double> &map_waypoints_x,
                             vector<double> &map_waypoints_y);
  void CalculateCost(Trajectory &trajectory);
  float SpeedCost(Trajectory &trajectory);
  float LaneChangeCost(Trajectory &trajectory);
  float AvoidColisionCost(Trajectory &trajectory);
  float BufferDistanceCost(Trajectory &trajectory);

  bool CheckColision();
  double CalculateLaneSpeed(int lane);

  vector<vector<double>> PredictSensorFusion();
  vector<double> PredictVehicle();
  vector<double> GetVehicleAhead(int lane);
  vector<double> GetVehicleBehind(int lane);

  void PrintStatistics();

 private:
  double x_{0};
  double y_{0};
  double s_{0};
  double d_{0};
  double yaw_{0};
  double speed_{0};
  double lane_{0};
  nlohmann::basic_json<> sensor_fusion_{};
  nlohmann::basic_json<> previous_path_x_{};
  nlohmann::basic_json<> previous_path_y_{};
  double end_path_s_{0};
  double end_path_d_{0};

  State state_{State::R};
  State state_prev_{State::R};

  double trajectory_final_speed_{0};
  double trajectory_final_speed_prev_{0};
  double target_speed_{0};
  double target_lane_{0};
  double target_cost_{0};
  double target_trajectory_len_{30};
  bool keep_trajectory_{false};

  vector<vector<double>> predictions_{};
  vector<int> lane_speed_predictions_{};
  vector<int> lane_speed_{};

  float prediction_dist_prev_{0.0};
  // trajectory ?

  // Limits and constants
  double safe_s_dist{15.0};
  double safe_s_dist_ahead_{15.0};
  double safe_s_dist_behind_{5.0};
  double speed_limit_mph_{49.5};
  double speed_buffer_mph_{0.5};
  double acc_limit_{10};
  int num_lanes_{3};
  double time_step_{0.02};
  double lane_width_{4};
  int trajectory_buffer_size_{50};
  double vehicle_length_{5.0};

  // Generate predictions for all other vehicles
  // choose_next State(predictions)
  // Realize next state
};

#endif

/*

#ifndef VEHICLE_H
#define VEHICLE_H

#include <map>
#include <string>
#include <vector>

using std::map;
using std::string;
using std::vector;

class Vehicle {
 public:
  // Constructors
  Vehicle();
  Vehicle(int lane, float s, float v, float a, string state="CS");

  // Destructor
  virtual ~Vehicle();

  // Vehicle functions
  vector<Vehicle> choose_next_state(map<int, vector<Vehicle>> &predictions);

  vector<string> successor_states();

  vector<Vehicle> generate_trajectory(string state,
                                      map<int, vector<Vehicle>> &predictions);

  vector<float> get_kinematics(map<int, vector<Vehicle>> &predictions, int
lane);

  vector<Vehicle> constant_speed_trajectory();

  vector<Vehicle> keep_lane_trajectory(map<int, vector<Vehicle>> &predictions);

  vector<Vehicle> lane_change_trajectory(string state,
                                         map<int, vector<Vehicle>>
&predictions);

  vector<Vehicle> prep_lane_change_trajectory(string state,
                                              map<int, vector<Vehicle>>
&predictions);

  void increment(int dt);

  float position_at(int t);

  bool get_vehicle_behind(map<int, vector<Vehicle>> &predictions, int lane,
                          Vehicle &rVehicle);

  bool get_vehicle_ahead(map<int, vector<Vehicle>> &predictions, int lane,
                         Vehicle &rVehicle);

  vector<Vehicle> generate_predictions(int horizon=2);

  void realize_next_state(vector<Vehicle> &trajectory);

  void configure(vector<int> &road_data);

  // public Vehicle variables
  struct collider{
    bool collision; // is there a collision?
    int  time; // time collision happens
  };

  map<string, int> lane_direction = {{"PLCL", 1}, {"LCL", 1},
                                     {"LCR", -1}, {"PLCR", -1}};

  int L = 1;

  int preferred_buffer = 6; // impacts "keep lane" behavior.

  int lane, s, goal_lane, goal_s, lanes_available;

  float v, target_speed, a, max_acceleration;

  string state;
};

#endif  // VEHICLE_H
*/