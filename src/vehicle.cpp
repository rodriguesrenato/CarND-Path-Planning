
#include "vehicle.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <iterator>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "helpers.h"
#include "spline.h"

using std::string;
using std::vector;

// Initialize Vehicle with configurations of feature limits and simulator
Vehicle::Vehicle(double road_speed_limit, double acc_limit, int num_lanes,
                 double time_step, double lane_width) {
  speed_limit_mph_ = road_speed_limit - speed_buffer_mph_;
  acc_limit_ = acc_limit;
  num_lanes_ = num_lanes;
  time_step_ = time_step;
  lane_width_ = lane_width;  // TODO: Trow an exception on lane_width < 0
}

// Define new state
void Vehicle::SetState(State state) {
  state_prev_ = state_;
  state_ = state;
}

// Set current vehicle localization variables
void Vehicle::SetLocalizationData(double x, double y, double s, double d,
                                  double yaw, double speed_mph) {
  x_ = x;
  y_ = y;
  s_ = s;
  d_ = d;
  lane_ = CalculateLaneIndex(d);
  yaw_ = yaw;
  speed_ = speed_mph / 2.237;  // TODO: Wrap in a function
}

// Set previous path
void Vehicle::SetPrevPath(nlohmann::basic_json<> previous_path_x,
                          nlohmann::basic_json<> previous_path_y,
                          double end_path_s, double end_path_d) {
  previous_path_x_ = previous_path_x;
  previous_path_y_ = previous_path_y;
  end_path_s_ = end_path_s;
  end_path_d_ = end_path_d;
}

void Vehicle::SetSensorFusion(nlohmann::basic_json<> sensor_fusion) {
  sensor_fusion_ = sensor_fusion;
  predictions_.clear();
  predictions_2.clear();
  lane_speed_.clear();
}
// Convert d to lane index
int Vehicle::CalculateLaneIndex(double d) {
  if (d >= 0) {
    return d / lane_width_;
  } else {
    return d / lane_width_ - 1;
  }
}

vector<int> Vehicle::LanesToCheckSpeed(double d) {
  vector<int> lanes;
  if (d >= 0) {
    lanes.push_back(d / lane_width_);
  } else {
    lanes.push_back(d / lane_width_ - 1);
  }
  float lane_pos = fmod(d, lane_width_);
  if (lane_pos < (vehicle_width_ / 2)) {
    lanes.push_back(lanes[0] - 1);
  }
  if (lane_pos > lane_width_ - (vehicle_width_ / 2)) {
    lanes.push_back(lanes[0] + 1);
  }
  return lanes;
}

void Vehicle::SetChosenTrajectory(Trajectory &trajectory) {
  target_speed_ = trajectory.target_speed;  // TODO: Debug
  trajectory_final_speed_ = trajectory.final_speed;
  target_lane_ = trajectory.target_lane;  // TODO: Debug
  projected_lane_ = trajectory.projected_lane;
  target_cost_ = trajectory.cost;
  SetState(trajectory.state);
}

vector<State> Vehicle::GetSuccessorStates() {
  vector<State> states;
  states.push_back(State::KL);
  switch (state_) {
    case State::KL:
      if (lane_ > 0) {
        states.push_back(State::PLCL);
      }
      if (lane_ < num_lanes_ - 1) {
        states.push_back(State::PLCR);
      }
      break;
    case State::PLCL:
      if (lane_ > 0) {
        states.push_back(State::LCL);
        states.push_back(State::PLCL);
      }
      break;
    case State::PLCR:
      if (lane_ < num_lanes_) {
        states.push_back(State::LCR);
        states.push_back(State::PLCR);
      }
      break;
    case State::LCL:
      // TODO: VERIFY END OF LAANE CHANGE WHEN d_ - d_of_target_lane is half
      // meter or close enough
      if (lane_ != target_lane_) {
        states.push_back(State::LCL);
      }
      break;
    case State::LCR:
      if (lane_ != target_lane_) {
        states.push_back(State::LCR);
      }
      break;
  }
  return states;
}

// pred_sensor_fusion =  [ id, s,final_s, d, final_speed]
vector<vector<double>> Vehicle::PredictSensorFusion(int buffer_size) {
  if (predictions_.find(buffer_size) == predictions_.end()) {
    vector<vector<double>> predictions;
    for (int i = 0; i < sensor_fusion_.size(); i++) {
      double vx = sensor_fusion_[i][3];
      double vy = sensor_fusion_[i][4];
      double check_speed = sqrt(vx * vx + vy * vy);
      double check_car_s = sensor_fusion_[i][5];
      // Project s value outwards in time
      check_car_s += ((double)buffer_size * time_step_ * check_speed);

      predictions.push_back({sensor_fusion_[i][0], sensor_fusion_[i][5],
                             check_car_s, sensor_fusion_[i][6], check_speed});
    }
    predictions_[buffer_size] = predictions;
  }
  return predictions_[buffer_size];
}

Trajectory Vehicle::GenerateTrajectory(State state,
                                       vector<double> &map_waypoints_s,
                                       vector<double> &map_waypoints_x,
                                       vector<double> &map_waypoints_y) {
  Trajectory trajectory;
  if (state == State::KL) {
    trajectory = GenerateTrajectoryKL2(target_trajectory_len_, map_waypoints_s,
                                       map_waypoints_x, map_waypoints_y);
  } else if (state == State::PLCL || state == State::PLCR) {
    trajectory =
        GenerateTrajectoryPLC2(state, target_trajectory_len_, map_waypoints_s,
                               map_waypoints_x, map_waypoints_y);
  } else if (state == State::LCL || state == State::LCR) {
    trajectory =
        GenerateTrajectoryLC2(state, target_trajectory_len_, map_waypoints_s,
                              map_waypoints_x, map_waypoints_y);
  }
  return trajectory;
}

Trajectory Vehicle::GenerateTrajectoryKL2(double target_traj_len,
                                          vector<double> &map_waypoints_s,
                                          vector<double> &map_waypoints_x,
                                          vector<double> &map_waypoints_y) {
  return BuildTrajectory(State::KL, lane_, trajectory_buffer_size_,
                         target_traj_len, map_waypoints_s, map_waypoints_x,
                         map_waypoints_y);
}

Trajectory Vehicle::GenerateTrajectoryPLC2(State state, double target_traj_len,
                                           vector<double> &map_waypoints_s,
                                           vector<double> &map_waypoints_x,
                                           vector<double> &map_waypoints_y) {
  int target_lane =
      lane_ + (state == State::PLCR) * 1 + (state == State::PLCL) * -1;

  return BuildTrajectory(state, target_lane, trajectory_buffer_size_,
                         target_traj_len, map_waypoints_s, map_waypoints_x,
                         map_waypoints_y);
}

Trajectory Vehicle::GenerateTrajectoryLC2(State state, double target_traj_len,
                                          vector<double> &map_waypoints_s,
                                          vector<double> &map_waypoints_x,
                                          vector<double> &map_waypoints_y) {
  int target_lane =
      lane_ + (state == State::LCR) * 1 + (state == State::LCL) * -1;
  return BuildTrajectory(state, target_lane, trajectory_buffer_size_,
                         target_traj_len, map_waypoints_s, map_waypoints_x,
                         map_waypoints_y);
}

// Returns a vector [final_speed,final_distance,final_s ] of the closest vehicle
// ahead
vector<double> Vehicle::GetVehicleAhead(int lane) {
  double distance_ahead = safe_s_dist_ahead_ + vehicle_length_;
  // preds = [ id, s,final_s, d, final_speed]
  auto preds = PredictSensorFusion(previous_path_x_.size());
  vector<double> vehicle_ahead{};
  for (int i = 0; i < preds.size(); i++) {
    if (lane == CalculateLaneIndex(preds[i][3])) {
      if (preds[i][1] > s_ && preds[i][2] > end_path_s_ &&
          (preds[i][2] - end_path_s_) < distance_ahead) {
        distance_ahead = preds[i][2] - end_path_s_;
        vehicle_ahead = {preds[i][4], distance_ahead, preds[i][2]};
      }
    }
  }
  // Possibly get the slowest speed within the safe distance, but it will only
  // worth to check if the front car will hit the car in front of it
  return vehicle_ahead;
}

// Returns a vector [final_speed,final_distance,final_s ] of the closest vehicle
// behind
vector<double> Vehicle::GetVehicleBehind(int lane) {
  double distance_behind = safe_s_dist_behind_ + vehicle_length_;
  // preds = [ id, s,final_s, d, final_speed]
  auto preds = PredictSensorFusion(previous_path_x_.size());
  vector<double> vehicle_behind{};
  for (int i = 0; i < preds.size(); i++) {
    if (lane == CalculateLaneIndex(preds[i][3])) {
      if (preds[i][1] < s_ && preds[i][2] < end_path_s_ &&
          (end_path_s_ - preds[i][2]) < distance_behind) {
        distance_behind = end_path_s_ - preds[i][2];
        vehicle_behind = {preds[i][4], distance_behind, preds[i][2]};
      }
    }
  }
  return vehicle_behind;
}

Trajectory Vehicle::BuildTrajectory(State state, int target_lane, int traj_size,
                                    double traj_s_len,
                                    vector<double> &map_waypoints_s,
                                    vector<double> &map_waypoints_x,
                                    vector<double> &map_waypoints_y) {
  // Define the actual x and y points we will use for the planner
  Trajectory trajectory;

  if (traj_size < 3) {
    return trajectory;
  }

  // Check for the lowest safe speed at the current lane position
  vector<double> speeds{};
  vector<int> lanes = LanesToCheckSpeed(d_);
  if (std::find(lanes.begin(), lanes.end(), target_lane) != lanes.end()) {
    lanes.push_back(target_lane);
  }

  for (int i = 0; i < lanes.size(); i++) {
    double lane_speed = speed_limit_mph_ / MPH_TO_MS;
    auto vehicle_ahead = GetVehicleAhead(lanes[i]);
    auto vehicle_behind = GetVehicleBehind(lanes[i]);

    if (!vehicle_ahead.empty()) {
      if (!vehicle_behind.empty()) {
        lane_speed = vehicle_ahead[0];
      } else {
        double speed_adj =
            vehicle_ahead[0] *
            ((vehicle_ahead[1] /
              (safe_s_dist_ahead_ + vehicle_length_)));  // *0.1 +0.9
        lane_speed = std::min(vehicle_ahead[0], speed_adj);
      }
    }
    speeds.push_back(lane_speed);
  }

  // Set the minimum speed as the build trajectory speed
  double speed = *std::min_element(speeds.begin(), speeds.end());

  // Set the trajectory final lane
  int projected_lane =
      lane_ + (state == State::LCR) * 1 + (state == State::LCL) * -1;

  int prev_path_size = previous_path_x_.size();

  // Check if trajectory needs to be reconstructed near the beginning due to
  // rapid change on speed or start a LC
  bool lane_change_state_check = (state != State::LCL) || (state != State::LCR);

  if ((speed_ - speed) > 2.0 || lane_change_state_check) {
    // Use only 20% of the previous points
    if (prev_path_size > traj_size / 10) {
      prev_path_size = traj_size / 10;
    }
  }
  // Define the anchor points to create the spline
  vector<double> ptsx{};
  vector<double> ptsy{};

  double ref_x = x_;
  double ref_y = y_;
  double ref_yaw = deg2rad(yaw_);

  // If previous size is almost empty, use the car as starting point
  if (prev_path_size < 2) {
    // prev_path_size = 0;
    // Use two points that make the path tangent to the car
    double prev_car_x = ref_x - cos(ref_yaw);
    double prev_car_y = ref_y - sin(ref_yaw);

    ptsx.push_back(prev_car_x);
    ptsx.push_back(ref_x);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(ref_y);
    std::cout << "ref_x= " << ref_x << "\tref_y= " << ref_y << std::endl;
  }

  // Use the previous path's end points as starting reference
  else {
    // Redefine reference state as previous path end point
    ref_x = previous_path_x_[prev_path_size - 1];
    ref_y = previous_path_y_[prev_path_size - 1];

    double ref_x_prev = previous_path_x_[prev_path_size - 2];
    double ref_y_prev = previous_path_y_[prev_path_size - 2];
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

    // Use two points that make the path tangent to the previous
    // path's end points
    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);

    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }

  // In frenet add evenly 30m spaced points ahead of the starting
  // reference
  // TODO traj_s_len*1.5
  vector<double> next_wp0 =
      getXY(s_ + 30, 2 + 4 * projected_lane, map_waypoints_s, map_waypoints_x,
            map_waypoints_y);
  vector<double> next_wp1 =
      getXY(s_ + 45, 2 + 4 * projected_lane, map_waypoints_s, map_waypoints_x,
            map_waypoints_y);
  vector<double> next_wp2 =
      getXY(s_ + 60, 2 + 4 * projected_lane, map_waypoints_s, map_waypoints_x,
            map_waypoints_y);

  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);

  // At this point, we have 5 reference points
  // Shift car reference angle to 0 degrees to help in further calculations
  for (int i = 0; i < ptsx.size(); i++) {
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;

    ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
    ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
  }

  // Create a spline
  tk::spline s;

  // Set x and y points to the spline
  s.set_points(ptsx, ptsy);

  // Start with all of the previous path points from last time TODO: Jogar pra
  // cima
  for (int i = 0; i < prev_path_size; i++) {
    trajectory.x.push_back(previous_path_x_[i]);
    trajectory.y.push_back(previous_path_y_[i]);
  }

  // Constrain speed to maximum speed
  if (speed > speed_limit_mph_ / MPH_TO_MS) {
    speed = speed_limit_mph_ / MPH_TO_MS;
  }

  double trajectory_final_speed =
      trajectory_final_speed_;  // TODO trocar para prev_

  // Fazer um check do speed do inicio da trajetoria pro speed final se delta_v
  // nao eh maior que a maxima aceleracao delva_v = a*delta_t ; delta_t =1, pois
  // o novo speed ira ser o mesmo ate o final do buffer pra dar 1segundo. Caso
  // contrario, mantem o ultimo speed ate que mude o inial. Acho que nem vai
  // precisar porque se cai para 5 o buffer antigo. vai ter no maixmo umas duas
  // iteracoes (2-3 pos consumidas por step), assim o daria para acelerar no
  // maximo 2*.224=.448
  // double max_acceleration = 10;
  // if (fabs(trajectory_final_speed - speed) < max_acceleration) {
  if (trajectory_final_speed > speed) {
    trajectory_final_speed -= .224;
  } else {
    if (trajectory_final_speed < speed) {
      trajectory_final_speed += .224;
    }
  }
  // }

  // Calculate how to break up spline points so that we travel at our
  // desired reference velocity
  double target_x = traj_s_len;  // TODO: Refactor target_traj_len name
  double target_y = s(target_x);
  double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));
  double x_add_on = 0;

  for (int i = trajectory.x.size(); i < traj_size; i++) {
    double N = (target_dist / (time_step_ * trajectory_final_speed));
    double x_point = x_add_on + (target_x) / N;

    double y_point = s(x_point);

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    // Rotate back to normal after rotating it earlier
    x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
    y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

    x_point += ref_x;
    y_point += ref_y;

    trajectory.x.push_back(x_point);
    trajectory.y.push_back(y_point);
  }

  // Buffering to be used in next trajectory build
  trajectory.prev_speed = trajectory_final_speed_;
  trajectory.final_speed = trajectory_final_speed;
  trajectory.state = state;
  trajectory.prev_state = state_;

  // Vars to calc cost
  trajectory.target_speed = speed;
  trajectory.target_lane = target_lane;
  trajectory.projected_lane = projected_lane;

  trajectory.lane_speed = speed;
  double final_x = trajectory.x[traj_size - 1];
  double final_y = trajectory.y[traj_size - 1];
  double final_x_prev = trajectory.x[traj_size - 2];
  double final_y_prev = trajectory.y[traj_size - 2];
  double final_yaw = atan2(final_y - final_y_prev, final_x - final_x_prev);

  auto final_sd =
      getFrenet(final_x, final_y, final_yaw, map_waypoints_x, map_waypoints_y);
  trajectory.final_s = final_sd[0];
  trajectory.final_d = final_sd[1];
  trajectory.final_lane = CalculateLaneIndex(final_sd[1]);
  return trajectory;
}

void Vehicle::CalculateCost(Trajectory &trajectory) {
  vector<float> weights{1.0, 1.0, 1000.0, 1.0};
  float cost0 = weights[0] * SpeedCost(trajectory);
  float cost1 = weights[1] * LaneChangeCost(trajectory);
  float cost2 = weights[2] * AvoidColisionCost(trajectory);
  float cost3 =
      weights[3] * BufferDistanceCost(
                       trajectory);  // TODO: remover, adicionar OutOfBoundaries
  trajectory.cost = cost0 + cost1 + cost2 + cost3;
  trajectory.cost_debug = {cost0, cost1, cost2, cost3};
}

float Vehicle::SpeedCost(Trajectory &trajectory) {
  float cost = 0;
  float stop_cost = 0.8;
  double speed = trajectory.target_speed;
  if (speed > (speed_limit_mph_ + speed_buffer_mph_) / MPH_TO_MS || speed < 0) {
    cost = 1;
  } else if (speed < speed_limit_mph_ / MPH_TO_MS) {
    cost = stop_cost * ((speed_limit_mph_ / MPH_TO_MS) - speed) /
           (speed_limit_mph_ / MPH_TO_MS);
  } else {
    cost = (speed - speed_limit_mph_ / MPH_TO_MS) /
           (speed_buffer_mph_ / MPH_TO_MS);
  }

  return cost;
}

float Vehicle::LaneChangeCost(Trajectory &trajectory) {
  // [final_speed,final_distance,final_s ]
  float optimal_speed = speed_limit_mph_ / MPH_TO_MS;

  double projected_speed = CalculateLaneSpeed(trajectory.projected_lane);
  double target_speed = CalculateLaneSpeed(trajectory.target_lane);
  float cost =
      ((2.0 * optimal_speed) - projected_speed - target_speed) / optimal_speed;
  return cost;
}

double Vehicle::CalculateLaneSpeed(int lane) {
  if (lane_speed_.find(lane) == lane_speed_.end()) {
    double lane_speed = speed_limit_mph_ / MPH_TO_MS;
    auto preds = PredictSensorFusion(previous_path_x_.size());
    // preds[ id, s,final_s, d, final_speed]
    for (int i = 0; i < preds.size(); i++) {
      if (lane == CalculateLaneIndex(
                      preds[i][3])) {  // adicionar o vehicle length? TODO
        if (preds[i][1] > s_ && preds[i][1] < s_ + target_trajectory_len_) {
          if (preds[i][4] < lane_speed) {
            lane_speed = preds[i][4];
          }
        }
      }
    }
    lane_speed_[lane] = lane_speed;
  }
  return lane_speed_[lane];
}

float Vehicle::AvoidColisionCost(Trajectory &trajectory) {
  return CheckColision(s_, trajectory.final_s, lane_, trajectory.final_lane,
                       trajectory_buffer_size_);
}

bool Vehicle::CheckColision(double init_s, double final_s, int init_lane,
                            int final_lane, int buffer_size) {
  auto predictions = PredictSensorFusion(buffer_size);
  bool colision = false;
  double threshold_upper_final_s = final_s + vehicle_length_;
  double threshold_lower_final_s = final_s - vehicle_length_;
  double threshold_upper_init_s = init_s + vehicle_length_;
  double threshold_lower_init_s = init_s - vehicle_length_;
  // double threshold_upper_final_s = final_s + safe_s_dist_ahead_;
  //   double threshold_lower_final_s = final_s - safe_s_dist_behind_;
  //   double threshold_upper_init_s = init_s + safe_s_dist_ahead_;
  //   double threshold_lower_init_s = init_s - safe_s_dist_behind_;
  // preds[ id, s,final_s, d, final_speed]
  for (int i = 0; i < predictions.size(); i++) {
    int prediction_lane = CalculateLaneIndex(predictions[i][3]);
    if (prediction_lane == final_lane) {
      // 1. Check if there is a car besides
      if (predictions[i][1] > threshold_lower_init_s &&
          predictions[i][1] < threshold_upper_init_s) {
        colision = true;
        std::cout << "C1-";
      }

      // 2. Check if there will be a vehicle in the end path position
      if (predictions[i][2] > threshold_lower_final_s &&
          predictions[i][2] < threshold_upper_final_s) {
        colision = true;
        std::cout << "C2-";
      }
      // 3. Check the case where there is a vehicle too fast passing in that
      // lane
      if (predictions[i][1] < threshold_lower_init_s &&
          predictions[i][2] > threshold_upper_final_s) {
        colision = true;
        std::cout << "C3-";
      }

      // 4. Check the case where there is a vehicle too slow passing in that
      // lane
      if (predictions[i][1] > threshold_upper_final_s &&
          predictions[i][2] < threshold_lower_init_s) {
        colision = true;
        std::cout << "C4-";
      }

      // 4. Check the case where there is a vehicle at the middle of
      // the trajectory
      if (predictions[i][1] > threshold_upper_init_s &&
          predictions[i][2] < threshold_lower_final_s) {
        colision = true;
        std::cout << "C5-";
      }
    }
  }

  return colision;
}

float Vehicle::BufferDistanceCost(Trajectory &trajectory) { return 0.0; }
// float Vehicle::StateCost(Trajectory &trajectory) { return 0.0; }

void Vehicle::PrintStatistics(vector<Trajectory> t) {
  if (state_ != state_prev_) {
    vector<string> state_str{"R", "KL", "LCL", "PLCL", "LCR", "PLCR"};
    std::cout << std::fixed << std::setprecision(2)
              << "State: " << state_str[state_prev_] << "\t-> "
              << state_str[state_]
              // << "\tcost = " << target_cost_
              << "\tSpeed: " << speed_ << " -> " << trajectory_final_speed_
              << " | " << target_speed_ << std::fixed << std::setprecision(0)
              << "\tLane: " << lane_ << "->" << projected_lane_ << "|"
              << target_lane_ << "\t" << std::fixed << std::setprecision(3);
    for (int i = 0; i < t.size(); i++) {
      std::cout << state_str[t[i].state] << "[";
      for (int j = 0; j < t[i].cost_debug.size(); j++) {
        std::cout << t[i].cost_debug[j] << "|";
      }
      std::cout << "] ";
    }
    std::cout << std::endl;
  }
}