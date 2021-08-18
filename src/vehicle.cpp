
#include "vehicle.h"

#include <algorithm>
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
  prediction_dist_prev_ = -1;
}
// Convert d to lane index
int Vehicle::CalculateLaneIndex(double d) {
  if (d >= 0) {
    return d / lane_width_;
  } else {
    return d / lane_width_ - 1;
  }
}
void Vehicle::SetChosenTrajectory(Trajectory &trajectory) {
  target_speed_ = trajectory.target_speed;
  trajectory_final_speed_ = trajectory.final_speed;
  target_lane_ = trajectory.final_lane;
  target_cost_ = trajectory.cost;
  SetState(trajectory.state);
}

vector<State> Vehicle::GetSuccessorStates() {
  vector<State> states;
  states.push_back(State::KL);
  switch (state_) {
    case State::KL:
      if (lane_ > 0) {
        states.push_back(State::LCL);
      }
      if (lane_ < num_lanes_-1) {
        states.push_back(State::LCR);
      }
      break;
    case State::PLCL:
      if (lane_ > 0) {
        states.push_back(State::PLCL);
        states.push_back(State::LCL);
      }
      break;
    case State::PLCR:
      if (lane_ < num_lanes_) {
        states.push_back(State::PLCR);
        states.push_back(State::LCR);
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

// pred_sensor_fusion =  [ id, s, d,final_s,final_speed]
vector<vector<double>> Vehicle::PredictSensorFusion(float pred_dist) {
  if (prediction_dist_prev_ != pred_dist) {
    predictions_.clear();
    for (int i = 0; i < sensor_fusion_.size(); i++) {
      double vx = sensor_fusion_[i][3];
      double vy = sensor_fusion_[i][4];
      double check_speed = sqrt(vx * vx + vy * vy);
      double check_car_s = sensor_fusion_[i][5];
      // Project s value outwards in time
      check_car_s += ((double)pred_dist * time_step_ * check_speed);

      predictions_.push_back({sensor_fusion_[i][0], sensor_fusion_[i][5],
                              sensor_fusion_[i][6], check_car_s, check_speed});
    }
    prediction_dist_prev_ = pred_dist;
  }
  return predictions_;
}

Trajectory Vehicle::GenerateTrajectory(State state,
                                       vector<double> &map_waypoints_s,
                                       vector<double> &map_waypoints_x,
                                       vector<double> &map_waypoints_y) {
  Trajectory trajectory;
  if (state == State::KL) {
    trajectory = GenerateTrajectoryKL(target_trajectory_len_, map_waypoints_s,
                                      map_waypoints_x, map_waypoints_y);
  } else if (state == State::LCL || state == State::LCR) {
    trajectory =
        GenerateTrajectoryLC(state, target_trajectory_len_, map_waypoints_s,
                             map_waypoints_x, map_waypoints_y);
  } else if (state == State::PLCL || state == State::PLCR) {
    // trajectory =
    //     GenerateTrajectoryPLC(state, target_trajectory_len, map_waypoints_s,
    //                           map_waypoints_x, map_waypoints_y);
    trajectory = GenerateTrajectoryKL(target_trajectory_len_, map_waypoints_s,
                                      map_waypoints_x, map_waypoints_y);
  }
  return trajectory;
}

Trajectory Vehicle::GenerateTrajectoryKL(double target_traj_len,
                                         vector<double> &map_waypoints_s,
                                         vector<double> &map_waypoints_x,
                                         vector<double> &map_waypoints_y) {
  // Define the actual x and y points we will use for the planner
  vector<double> next_x_vals{};
  vector<double> next_y_vals{};
  Trajectory trajectory;

  // Define the anchor points to create the spline
  vector<double> ptsx;
  vector<double> ptsy;

  double ref_x = x_;
  double ref_y = y_;
  double ref_yaw = deg2rad(yaw_);

  int prev_path_size = previous_path_x_.size();

  // Check if there is any car ahead
  double target_speed = speed_limit_mph_ / MPH_TO_MS;
  double pred_dist_car_ahead = 0;
  auto preds = PredictSensorFusion(target_traj_len);

  // sort by distance preds[ id, s, d,final_s,final_speed]
  int car_ahead_index = -1;
  for (int i = 0; i < preds.size(); i++) {
    if (lane_ == CalculateLaneIndex(preds[i][2])) {
      if (s_ < preds[i][1] && (end_path_s_ + safe_s_dist) > preds[i][3]) {
        if (car_ahead_index < 0) {
          car_ahead_index = i;
        } else {
          if (preds[i][1] < preds[car_ahead_index][1]) {
            car_ahead_index = i;
          }
        }
      }
    }
  }

  // If prediction found a car that will be ahead less than the safe distance,
  // then adjust speed to the car ahead speed. If there isn't any car ahead, the
  // target speed will be the MAX SPEED LIMIT
  if (car_ahead_index >= 0) {
    // std::cout << "Slower car found on lane " << preds[car_ahead_index][2]
    //           << " , s_/pred= " << s_ << " / " << preds[car_ahead_index][1]
    //           << " , end_s/pred= " << end_path_s_ << " / " <<
    //           preds[car_ahead_index][3]
    //           << " , speed_/pred= " << speed_ << " / " <<
    //           preds[car_ahead_index][4]
    //           << " , safe_s_dist= " << safe_s_dist
    //           << std::endl;
    pred_dist_car_ahead = preds[car_ahead_index][3] - end_path_s_;
    target_speed = preds[car_ahead_index][4];
  }

  // TODO: Define this accordingly with dist from next car or velocity changes
  if (fabs(target_speed - speed_) > 2.0) {
    prev_path_size /= 2;
  }
  // If previous size is almost empty, use the car as starting point
  if (prev_path_size < 2) {
    // Use two points that make the path tangent to the car
    double prev_car_x = ref_x - cos(ref_yaw);
    double prev_car_y = ref_y - sin(ref_yaw);

    ptsx.push_back(prev_car_x);
    ptsx.push_back(ref_x);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(ref_y);
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
  vector<double> next_wp0 = getXY(s_ + 30, 2 + 4 * lane_, map_waypoints_s,
                                  map_waypoints_x, map_waypoints_y);
  vector<double> next_wp1 = getXY(s_ + 60, 2 + 4 * lane_, map_waypoints_s,
                                  map_waypoints_x, map_waypoints_y);
  vector<double> next_wp2 = getXY(s_ + 90, 2 + 4 * lane_, map_waypoints_s,
                                  map_waypoints_x, map_waypoints_y);

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

  // Start with all of the previous path points from last time
  for (int i = 0; i < prev_path_size; i++) {
    next_x_vals.push_back(previous_path_x_[i]);
    next_y_vals.push_back(previous_path_y_[i]);
  }

  if (target_speed > speed_limit_mph_ / MPH_TO_MS) {
    target_speed = speed_limit_mph_ / MPH_TO_MS;
  }
  double trajectory_final_speed_prev = trajectory_final_speed_;

  if (trajectory_final_speed_prev > target_speed) {
    trajectory_final_speed_prev -= .224;
  } else {
    if (trajectory_final_speed_prev < target_speed) {
      trajectory_final_speed_prev += .224;
    }
  }

  // Calculate how to break up spline points so that we travel at our
  // desired reference velocity
  double target_x = target_traj_len;  // TODO: Refactor target_traj_len name
  double target_y = s(target_x);
  double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));
  double x_add_on = 0;

  for (int i = next_x_vals.size(); i < trajectory_buffer_size_; i++) {
    double N = (target_dist / (0.02 * trajectory_final_speed_prev));
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

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }

  trajectory.x = next_x_vals;
  trajectory.y = next_y_vals;
  trajectory.prev_speed = trajectory_final_speed_;
  trajectory.prev_lane = lane_;
  trajectory.final_speed = trajectory_final_speed_prev;
  trajectory.final_lane = lane_;
  trajectory.target_speed = target_speed;
  trajectory.state = State::KL;

  double final_x = next_x_vals[trajectory_buffer_size_ - 1];
  double final_y = next_y_vals[trajectory_buffer_size_ - 1];
  double final_x_prev = next_x_vals[trajectory_buffer_size_ - 2];
  double final_y_prev = next_y_vals[trajectory_buffer_size_ - 2];
  double final_yaw = atan2(final_y - final_y_prev, final_x - final_x_prev);

  trajectory.final_s = getFrenet(final_x, final_y, final_yaw, map_waypoints_x,
                                 map_waypoints_y)[0];

  return trajectory;
}

Trajectory Vehicle::GenerateTrajectoryLC(State state, double target_traj_len,
                                         vector<double> &map_waypoints_s,
                                         vector<double> &map_waypoints_x,
                                         vector<double> &map_waypoints_y) {
  // Define the actual x and y points we will use for the planner
  vector<double> next_x_vals{};
  vector<double> next_y_vals{};
  Trajectory trajectory;

  // Define the anchor points to create the spline
  vector<double> ptsx;
  vector<double> ptsy;

  if (state == State::LCL) {
    target_lane_ = lane_ - 1;
  } else if (state == State::LCR) {
    target_lane_ = lane_ + 1;
  }

  double ref_x = x_;
  double ref_y = y_;
  double ref_yaw = deg2rad(yaw_);

  int prev_path_size = previous_path_x_.size()/2;

  // Check if there is any car ahead
  double target_speed = speed_limit_mph_ / MPH_TO_MS;
  double target_speed_final = speed_limit_mph_ / MPH_TO_MS;
  double pred_dist_car_ahead = 0;
  auto preds = PredictSensorFusion(target_traj_len);

  // sort by distance preds[ id, s, d,final_s,final_speed]
  int car_ahead_index = -1;
  int car_next_lane_index = -1;
  for (int i = 0; i < preds.size(); i++) {
    if (lane_ ==
        CalculateLaneIndex(preds[i][2])) {  // or add both lanes check TODO
      if (s_ < preds[i][1] && (end_path_s_ + safe_s_dist) > preds[i][3]) {
        if (car_ahead_index < 0) {
          car_ahead_index = i;
        } else {
          if (preds[i][1] < preds[car_ahead_index][1]) {
            car_ahead_index = i;
          }
        }
      }
    }
    if (target_lane_ == CalculateLaneIndex(preds[i][2])) {
      if (s_ < preds[i][1] && (end_path_s_ + safe_s_dist) > preds[i][3]) {
        if (car_next_lane_index < 0) {
          car_next_lane_index = i;
        } else {
          if (preds[i][1] < preds[car_next_lane_index][1]) {
            car_next_lane_index = i;
          }
        }
      }
    }
  }

  // If prediction found a car that will be ahead less than the safe distance,
  // then adjust speed to the car ahead speed. If there isn't any car ahead, the
  // target speed will be the MAX SPEED LIMIT
  if (car_ahead_index >= 0) {
    pred_dist_car_ahead = preds[car_ahead_index][3] - end_path_s_;
    target_speed = preds[car_ahead_index][4];
  }

  if (car_next_lane_index >= 0) {
    target_speed_final = preds[car_next_lane_index][4];
  }

  // TODO: Define this accordingly with dist from next car or velocity changes
  if (fabs(target_speed - speed_) > 2.0 && target_speed < speed_) {
    prev_path_size /= 2;
  }
  // If previous size is almost empty, use the car as starting point
  if (prev_path_size < 2) {
    // Use two points that make the path tangent to the car
    double prev_car_x = ref_x - cos(ref_yaw);
    double prev_car_y = ref_y - sin(ref_yaw);

    ptsx.push_back(prev_car_x);
    ptsx.push_back(ref_x);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(ref_y);
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
  vector<double> next_wp0 =
      getXY(s_ + 30, 2 + 4 * target_lane_, map_waypoints_s, map_waypoints_x,
            map_waypoints_y);
  vector<double> next_wp1 =
      getXY(s_ + 60, 2 + 4 * target_lane_, map_waypoints_s, map_waypoints_x,
            map_waypoints_y);
  vector<double> next_wp2 =
      getXY(s_ + 90, 2 + 4 * target_lane_, map_waypoints_s, map_waypoints_x,
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

  // Start with all of the previous path points from last time
  for (int i = 0; i < prev_path_size; i++) {
    next_x_vals.push_back(previous_path_x_[i]);
    next_y_vals.push_back(previous_path_y_[i]);
  }

  if (target_speed > speed_limit_mph_ / MPH_TO_MS) {
    target_speed = speed_limit_mph_ / MPH_TO_MS;
  }
  double trajectory_final_speed_prev = trajectory_final_speed_;

  if (trajectory_final_speed_prev > target_speed) {
    trajectory_final_speed_prev -= .224;
  } else {
    if (trajectory_final_speed_prev < target_speed) {
      trajectory_final_speed_prev += .224;
    }
  }

  // Calculate how to break up spline points so that we travel at our
  // desired reference velocity
  double target_x = target_traj_len;  // TODO: Refactor target_traj_len name
  double target_y = s(target_x);
  double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));
  double x_add_on = 0;

  for (int i = next_x_vals.size(); i < trajectory_buffer_size_; i++) {
    double N = (target_dist / (0.02 * trajectory_final_speed_prev));
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

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }

  trajectory.x = next_x_vals;
  trajectory.y = next_y_vals;
  trajectory.final_speed = trajectory_final_speed_prev;
  trajectory.final_lane = target_lane_;
  trajectory.prev_speed = trajectory_final_speed_;
  trajectory.prev_lane = lane_;
  trajectory.target_speed = target_speed_final;
  trajectory.state = state;

  double final_x = next_x_vals[trajectory_buffer_size_ - 1];
  double final_y = next_y_vals[trajectory_buffer_size_ - 1];
  double final_x_prev = next_x_vals[trajectory_buffer_size_ - 2];
  double final_y_prev = next_y_vals[trajectory_buffer_size_ - 2];
  double final_yaw = atan2(final_y - final_y_prev, final_x - final_x_prev);

  trajectory.final_s = getFrenet(final_x, final_y, final_yaw, map_waypoints_x,
                                 map_waypoints_y)[0];

  return trajectory;
}

Trajectory Vehicle::GenerateTrajectoryPLC(State state, double target_traj_len,
                                          vector<double> &map_waypoints_s,
                                          vector<double> &map_waypoints_x,
                                          vector<double> &map_waypoints_y) {
  Trajectory t;
  return t;
}

// Trajectory Vehicle::GenerateTrajectoryL(State state, double target_traj_len,
//                                          vector<double> &map_waypoints_s,
//                                          vector<double> &map_waypoints_x,
//                                          vector<double> &map_waypoints_y) {
//                                          }

void Vehicle::CalculateCost(Trajectory &trajectory) {
  vector<float> weights{10.0, 1.0, 1000.0, 1.0};

  float cost = weights[0] * SpeedCost(trajectory);
  cost += weights[1] * LaneChangeCost(trajectory);
  cost += weights[2] * AvoidColisionCost(trajectory);
  cost += weights[3] * BufferDistanceCost(trajectory);
  trajectory.cost = cost;
}

float Vehicle::SpeedCost(Trajectory &trajectory) {
  float cost = 0;
  float stop_cost = 0.8;

  if (trajectory.target_speed >
          (speed_limit_mph_ + speed_buffer_mph_) / MPH_TO_MS ||
      trajectory.target_speed < 0) {
    cost = 1;
  } else if (trajectory.target_speed < speed_limit_mph_ / MPH_TO_MS) {
    cost = stop_cost *
           ((speed_limit_mph_ / MPH_TO_MS) - trajectory.target_speed) /
           (speed_limit_mph_ / MPH_TO_MS);
  } else {
    cost = (trajectory.target_speed - speed_limit_mph_ / MPH_TO_MS) /
           (speed_buffer_mph_ / MPH_TO_MS);
  }

  return cost;
}

float Vehicle::LaneChangeCost(Trajectory &trajectory) {
  return fabs(lane_ - trajectory.final_lane);
}
float Vehicle::AvoidColisionCost(Trajectory &trajectory) {
  float car_lenght = 5.0;
  float cost = 0;

  double threshold_upper_final = trajectory.final_s + vehicle_length_ * 1.5;
  double threshold_lower_final = trajectory.final_s - vehicle_length_ * 1.5;

  double threshold_upper_init = s_ + vehicle_length_ * 1.5;
  double threshold_lower_init = s_ - vehicle_length_ * 1.5;

  auto predictions = PredictSensorFusion(target_trajectory_len_);
  // preds[ id, s, d,final_s,final_speed]
  for (int i = 0; i < predictions.size(); i++) {
    if (trajectory.final_lane == CalculateLaneIndex(predictions[i][2])) {
      // Check if vehicle will colide at end
      if (predictions[i][3] < threshold_upper_final &&
          predictions[i][1] > threshold_lower_init) {
        cost += 1.0;
      }
    }
  }
  return cost;
}

float Vehicle::BufferDistanceCost(Trajectory &trajectory) { return 0.0; }
// float Vehicle::StateCost(Trajectory &trajectory) { return 0.0; }

void Vehicle::PrintStatistics() {
  vector<string> state_str{"R", "KL", "PLCL", "LCL", "PLCR", "LCR"};
  std::cout << std::fixed << std::setprecision(2)
            << "State = " << state_str[state_prev_] << "->" << state_str[state_]
            << "\tcost = " << target_cost_
            << "\tspd/traj_f_spd/target_spd = " << speed_ << " | "
            << trajectory_final_speed_ << " | " << target_speed_
            << "\tlane/target_lane = " << lane_ << " | " << target_lane_
            << std::endl;
}
