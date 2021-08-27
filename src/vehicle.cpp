#include "vehicle.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "helpers.h"
#include "spline.h"

// Initialize Vehicle with configurations of feature limits and simulator
Vehicle::Vehicle(double road_speed_limit_mph, double speed_buffer_mph,
                 int num_lanes, double time_step, double lane_width,
                 double trajectory_s_projection, double safe_s_dist_ahead,
                 double safe_s_dist_behind, int trajectory_buffer_size,
                 double vehicle_length, double vehicle_width) {
  speed_buffer_ = speed_buffer_mph / MPH_TO_MS;
  speed_limit_ = (road_speed_limit_mph - speed_buffer_mph) / MPH_TO_MS;
  num_lanes_ = num_lanes;
  time_step_ = time_step;
  lane_width_ = lane_width;
  trajectory_s_projection_ = trajectory_s_projection;
  safe_s_dist_ahead_ = safe_s_dist_ahead;
  safe_s_dist_behind_ = safe_s_dist_behind;
  trajectory_buffer_size_ = trajectory_buffer_size;
  vehicle_length_ = vehicle_length;
  vehicle_width_ = vehicle_width;
}

// Update the vehicle localization attributes
void Vehicle::UpdateLocalizationData(double x, double y, double s, double d,
                                     double yaw, double speed_mph) {
  x_ = x;
  y_ = y;
  s_ = s;
  d_ = d;
  yaw_ = yaw;
  speed_ = speed_mph / MPH_TO_MS;
  lane_ = CalculateLaneIndex(d);
}

// Update the previous path class attributes
void Vehicle::UpdatePreviousPath(nlohmann::basic_json<> previous_path_x,
                                 nlohmann::basic_json<> previous_path_y,
                                 double end_path_s, double end_path_d) {
  previous_path_x_ = previous_path_x;
  previous_path_y_ = previous_path_y;
  end_path_s_ = end_path_s;
  end_path_d_ = end_path_d;
}

// Update the sensor_fusion class attribute and clear the lane_speed_ and
// predictions_ vectors
void Vehicle::UpdateSensorFusion(nlohmann::basic_json<> sensor_fusion) {
  sensor_fusion_ = sensor_fusion;
  predictions_.clear();
  lane_speed_.clear();
}

// Update the main car class private attributes that will be used in next update
void Vehicle::SetChosenTrajectory(Trajectory &trajectory) {
  target_speed_ = trajectory.target_speed;
  projected_speed_ = trajectory.projected_speed;
  trajectory_final_speed_ = trajectory.final_speed;
  target_lane_ = trajectory.target_lane;
  projected_lane_ = trajectory.projected_lane;
  trajectory_cost = trajectory.cost;
  state_prev_ = state_;
  state_ = trajectory.state;
}

// Return the sucessor states for the current state
vector<State> Vehicle::GetSuccessorStates() {
  vector<State> states;
  states.push_back(State::KL);
  switch (state_) {
      // Before add states, check the current lane to avoid states that will
      // result on lanes out of boundaries
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
      if (lane_ < num_lanes_ - 1) {
        states.push_back(State::LCR);
        states.push_back(State::PLCR);
      }
      break;
    case State::LCL:
      // When the vehicle enters in the target_lane, then force a KL state to
      // only change more one lane before a PLC. Same for LCR
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

// Return the trajectory for the given state
Trajectory Vehicle::GenerateTrajectory(State state,
                                       vector<double> &map_waypoints_s,
                                       vector<double> &map_waypoints_x,
                                       vector<double> &map_waypoints_y) {
  // Check the state and build the respective trajectory
  if (state == State::KL) {
    return BuildTrajectory(state, lane_, trajectory_buffer_size_,
                           trajectory_s_projection_, map_waypoints_s,
                           map_waypoints_x, map_waypoints_y);
  } else if (state == State::PLCL || state == State::PLCR) {
    int best_lane = CalculateBestLane(state);
    return BuildTrajectory(state, best_lane, trajectory_buffer_size_,
                           trajectory_s_projection_, map_waypoints_s,
                           map_waypoints_x, map_waypoints_y);
  } else if (state == State::LCL || state == State::LCR) {
    // int best_lane = CalculateBestLane(state);
    int best_lane =
        lane_ + (state == State::LCR) * 1 + (state == State::LCL) * -1;
    // Increase s projection on curves to avoid jerk
    return BuildTrajectory(state, best_lane, trajectory_buffer_size_,
                           trajectory_s_projection_ * 1.2, map_waypoints_s,
                           map_waypoints_x, map_waypoints_y);
  }
  // If an invalid state was given, then return an empty trajectory
  return Trajectory();
}

// Return a trajectory built from the given inputs
Trajectory Vehicle::BuildTrajectory(State state, int target_lane,
                                    int trajectory_buffer_size,
                                    double trajectory_s_projection,
                                    vector<double> &map_waypoints_s,
                                    vector<double> &map_waypoints_x,
                                    vector<double> &map_waypoints_y) {
  // Define the actual x and y points we will use for the planner
  Trajectory trajectory;

  // Avoid invalid buffer size
  if (trajectory_buffer_size < 3) {
    return trajectory;
  }

  // Get the lanes that lane speed need to be checked according to the current d
  // position
  vector<int> lanes = LanesToCheckSpeed(d_);

  // Add the target lane in case it isn't in the lanes vector TODO:CHECK
  int intended_lane = lane_ + (state == State::PLCR) * 1 +
                      (state == State::PLCL) * -1 + (state == State::LCR) * 1 +
                      (state == State::LCL) * -1;
  if (std::find(lanes.begin(), lanes.end(), intended_lane) == lanes.end()) {
    lanes.push_back(intended_lane);
  }

  // Check for the lowest safe speed at the current lane position
  vector<double> speeds{};
  for (int i = 0; i < lanes.size(); i++) {
    double lane_speed = speed_limit_;
    auto vehicle_ahead = GetVehicleAhead(lanes[i]);
    auto vehicle_behind = GetVehicleBehind(lanes[i]);

    if (!vehicle_ahead.empty()) {
      if (!vehicle_behind.empty()) {
        // If there is a vehicle behind, keep following the speed of the vehicle
        // ahead
        lane_speed = vehicle_ahead[0];
      } else {
        // If there isn't a vehicle behind, reduce speed to keep the safe
        // distance from the vehicle ahead. The new speed is proportional to the
        // distance from car ahead.
        double speed_adj =
            vehicle_ahead[0] *
            ((vehicle_ahead[1] / (safe_s_dist_ahead_ + vehicle_length_)));
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

  // Define the previous path size
  int prev_path_size = previous_path_x_.size();

  // Check if trajectory needs to be reconstructed near the beginning due to
  // rapid change on speed or a lane change procedure
  bool lane_change_state_check = (state != State::LCL) || (state != State::LCR);

  if ((speed_ - speed) > 2.0 || lane_change_state_check) {
    // Use only 20% of the previous points
    if (prev_path_size > trajectory_buffer_size * 0.2) {
      prev_path_size = trajectory_buffer_size * 0.2;
    }
  }

  // Append the prev_path_size previous x and y points from the previous path to
  // the trajectory
  for (int i = 0; i < prev_path_size; i++) {
    trajectory.x.push_back(previous_path_x_[i]);
    trajectory.y.push_back(previous_path_y_[i]);
  }

  // Define the anchor points to create the spline
  vector<double> ptsx{};
  vector<double> ptsy{};

  // Define x, y and yaw reference points
  double ref_x = x_;
  double ref_y = y_;
  double ref_yaw = deg2rad(yaw_);

  // If previous size is almost empty, use the car as starting point
  if (prev_path_size < 2) {
    // Project a point back the current position
    double prev_car_x = ref_x - cos(ref_yaw);
    double prev_car_y = ref_y - sin(ref_yaw);

    // Use two points that make the path tangent to the car orientation
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

  // Using Frenet coordinates, calculate evenly 30m spaced points ahead of the
  // starting reference on the projected_lane and convert them to XY coordinates
  vector<double> next_wp0 =
      getXY(s_ + trajectory_s_projection, 2 + 4 * projected_lane,
            map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp1 =
      getXY(s_ + trajectory_s_projection * 1.5, 2 + 4 * projected_lane,
            map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp2 =
      getXY(s_ + trajectory_s_projection * 2.0, 2 + 4 * projected_lane,
            map_waypoints_s, map_waypoints_x, map_waypoints_y);

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

  // Constrain speed to the speed limit
  if (speed > speed_limit_) {
    speed = speed_limit_;
  }

  // Get the final speed from the previous trajectory
  double trajectory_final_speed = trajectory_final_speed_;

  // Check whether is to accelerate or decelerate according to the target speed
  if (trajectory_final_speed > speed) {
    trajectory_final_speed -= .224;
  } else {
    if (trajectory_final_speed < speed) {
      trajectory_final_speed += .224;
    }
  }

  // Calculate how to break up spline points so that we travel at our
  // desired reference velocity
  double target_x = trajectory_s_projection;
  double target_y = s(target_x);
  double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));
  double x_add_on = 0;
  double N = (target_dist / (time_step_ * trajectory_final_speed));

  // Add x and y points to the trajectory accordingly with the new speed
  for (int i = trajectory.x.size(); i < trajectory_buffer_size; i++) {
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

    // Append the new x and y points to the trajectory vectors
    trajectory.x.push_back(x_point);
    trajectory.y.push_back(y_point);
  }

  // Add state information
  trajectory.state = state;
  trajectory.prev_state = state_;

  // Buffering to be used in next trajectory build
  trajectory.final_speed_prev = trajectory_final_speed_;
  trajectory.final_speed = trajectory_final_speed;

  // Vars to calc cost
  trajectory.target_speed = speed;
  trajectory.target_lane = target_lane;
  trajectory.projected_speed = speed;
  trajectory.projected_lane = projected_lane;

  // Calculate the final s and d values of the built trajectory
  double final_x = trajectory.x[trajectory_buffer_size - 1];
  double final_y = trajectory.y[trajectory_buffer_size - 1];
  double final_x_prev = trajectory.x[trajectory_buffer_size - 2];
  double final_y_prev = trajectory.y[trajectory_buffer_size - 2];
  double final_yaw = atan2(final_y - final_y_prev, final_x - final_x_prev);

  auto final_sd =
      getFrenet(final_x, final_y, final_yaw, map_waypoints_x, map_waypoints_y);

  trajectory.final_s = final_sd[0];
  trajectory.final_d = final_sd[1];

  // Return the built trajectory
  return trajectory;
}

// Calculate the total cost for the given trajectory
void Vehicle::CalculateCost(Trajectory &trajectory) {
  float cost0 = cost_weights_[0] * SpeedCost(trajectory);
  float cost1 = cost_weights_[1] * LaneChangeCost(trajectory);
  float cost2 = cost_weights_[2] * CollisionCost(trajectory);
  trajectory.cost = cost0 + cost1 + cost2;

  // Costs vector helps debug and fine tune the cost weights
  trajectory.costs = {cost0, cost1, cost2};
}

// Return the target speed cost
float Vehicle::SpeedCost(Trajectory &trajectory) {
  float cost = 0;
  float stop_cost = 0.8;

  double speed = trajectory.projected_speed;
  if (speed > (speed_limit_ + speed_buffer_) || speed < 0) {
    cost = 1;
  } else if (speed < speed_limit_) {
    cost = stop_cost * (speed_limit_ - speed) / (speed_limit_);
  } else {
    cost = (speed - speed_limit_) / (speed_buffer_);
  }

  return cost;
}

// Return the lane change cost
float Vehicle::LaneChangeCost(Trajectory &trajectory) {
  // [final_speed,final_distance,final_s ]
  double optimal_speed = speed_limit_;

  double projected_speed = CalculateLaneSpeed(trajectory.projected_lane);
  double target_speed = CalculateLaneSpeed(trajectory.target_lane);

  return ((2.0 * optimal_speed) - projected_speed - target_speed) /
         optimal_speed;
}

// Return the colision cost
float Vehicle::CollisionCost(Trajectory &trajectory) {
  return CheckCollision(s_, trajectory.final_s, lane_,
                        trajectory.projected_lane, trajectory_buffer_size_);
}

// Check for collisions with other vehicles on the target lane
bool Vehicle::CheckCollision(double init_s, double final_s, int init_lane,
                             int final_lane, int buffer_size) {
  // As the vehicle has width and length, define boundaries for checking on the
  // initial and final position. To simplify, it will only check for collision
  // on the initial and final s value on the target lane
  double threshold_upper_final_s = final_s + vehicle_length_;
  double threshold_lower_final_s = final_s - vehicle_length_;
  double threshold_upper_init_s = init_s + vehicle_length_;
  double threshold_lower_init_s = init_s - vehicle_length_;

  bool collision = false;
  auto predictions = PredictSensorFusion(buffer_size);

  // predictions vector: [ id, s,final_s, d, final_speed]
  for (int i = 0; i < predictions.size(); i++) {
    int prediction_lane = CalculateLaneIndex(predictions[i][3]);
    if (prediction_lane == final_lane) {
      // 1. Check if there is a car besides
      if (predictions[i][1] > threshold_lower_init_s &&
          predictions[i][1] < threshold_upper_init_s) {
        collision = true;
      }

      // 2. Check if there will be a vehicle in the end path position
      if (predictions[i][2] > threshold_lower_final_s &&
          predictions[i][2] < threshold_upper_final_s) {
        collision = true;
      }
      // 3. Check the case where there is a vehicle too fast passing in that
      // lane
      if (predictions[i][1] < threshold_lower_init_s &&
          predictions[i][2] > threshold_upper_final_s) {
        collision = true;
      }

      // 4. Check the case where there is a vehicle too slow passing in that
      // lane
      if (predictions[i][1] > threshold_upper_final_s &&
          predictions[i][2] < threshold_lower_init_s) {
        collision = true;
      }

      // 4. Check the case where there is a vehicle at the middle of
      // the trajectory
      if (predictions[i][1] > threshold_upper_init_s &&
          predictions[i][2] < threshold_lower_final_s) {
        collision = true;
      }
    }
  }

  return collision;
}

// Predict the future positions and speed of each detected vehicle from sensor
// fusion and returns a vector of vehicle predictions vector, according to the
// given buffer size. [[ id, s,final_s, d, final_speed]]
vector<vector<double>> Vehicle::PredictSensorFusion(int buffer_size) {
  // Check if the predictions for the given buffer were already calculated. This
  // helps to avoid future unnecessary recalculations
  if (predictions_.find(buffer_size) == predictions_.end()) {
    vector<vector<double>> predictions;

    // Iterate over each sensor fusion measurements for a vehicle on the road
    for (int i = 0; i < sensor_fusion_.size(); i++) {
      double vx = sensor_fusion_[i][3];
      double vy = sensor_fusion_[i][4];
      double check_speed = sqrt(vx * vx + vy * vy);
      double check_car_s = sensor_fusion_[i][5];

      // Project s value outwards in time with the precalculated speed
      check_car_s += ((double)buffer_size * time_step_ * check_speed);

      // Add the custom vehicle vector of predictions and current positions
      predictions.push_back({sensor_fusion_[i][0], sensor_fusion_[i][5],
                             check_car_s, sensor_fusion_[i][6], check_speed});
    }
    // Update the predictions vector for the given buffer size
    predictions_[buffer_size] = predictions;
  }
  return predictions_[buffer_size];
}

// Return the correspondent lane for a give d frenet position
int Vehicle::CalculateLaneIndex(double d) {
  if (d >= 0) {
    return d / lane_width_;
  } else {
    return d / lane_width_ - 1;
  }
}

// Return the best lane according to the state and current lane
int Vehicle::CalculateBestLane(State state) {
  int best_lane = lane_;
  double lane_speed = 0.0;  // CalculateLaneSpeed(lane_);

  // If it is preparing a lane change, then check all lanes of the lane change
  // side. If it is a lane change, then check just the next lane according to
  // the lane change side
  if (state == State::PLCL) {
    for (int i = (lane_ - 1); i >= 0; i--) {
      double checked_speed = CalculateLaneSpeed(i);
      if (checked_speed > lane_speed) {
        best_lane = i;
      }
    }
  } else if (state == State::PLCR) {
    for (int i = (lane_ + 1); i < num_lanes_; i++) {
      double checked_speed = CalculateLaneSpeed(i);
      if (checked_speed > lane_speed) {
        best_lane = i;
      }
    }
  } else if (state == State::LCL || state == State::LCR) {
    int check_lane =
        lane_ + (state == State::LCR) * 1 + (state == State::LCL) * -1;
    double checked_speed = CalculateLaneSpeed(check_lane);
    if (checked_speed > lane_speed) {
      best_lane = check_lane;
    }
  }

  // Return the lane that has the highest speed
  return best_lane;
}

// Return the minimum speed on the given lane within the trajectory length
double Vehicle::CalculateLaneSpeed(int lane) {
  if (lane_speed_.find(lane) == lane_speed_.end()) {
    double lane_speed = speed_limit_;
    auto preds = PredictSensorFusion(previous_path_x_.size());

    // preds vector: [ id, s,final_s, d, final_speed]
    for (int i = 0; i < preds.size(); i++) {
      if (lane == CalculateLaneIndex(preds[i][3])) {
        if (preds[i][1] > s_ - vehicle_length_ &&
            preds[i][1] < s_ + vehicle_length_ + trajectory_s_projection_) {
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

// Return a vector of lanes that the vehicle is on the road according to the
// vehicle width
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

// Returns a vector of the closest vehicle ahead.
// [final_speed,final_distance,final_s ]
vector<double> Vehicle::GetVehicleAhead(int lane) {
  // Define the distance ahead with the maximum safe distance
  double distance_ahead = safe_s_dist_ahead_ + vehicle_length_;

  // Get the predition vector: [ id, s,final_s, d, final_speed]
  auto preds = PredictSensorFusion(previous_path_x_.size());

  // Iterate over each predicted vehicle that is on the same given lane
  vector<double> vehicle_ahead{};
  for (int i = 0; i < preds.size(); i++) {
    if (lane == CalculateLaneIndex(preds[i][3])) {
      if (preds[i][1] > s_ && preds[i][2] > end_path_s_ &&
          (preds[i][2] - end_path_s_) < distance_ahead) {
        // If a car was found ahead and closer to the main vehicle, then update
        // the distance_ahead and the vector vehicle_ahead with this vehicle
        // information
        distance_ahead = preds[i][2] - end_path_s_;
        vehicle_ahead = {preds[i][4], distance_ahead, preds[i][2]};
      }
    }
  }

  return vehicle_ahead;
}

// Returns a vector of the closest vehicle behind.
// [final_speed,final_distance,final_s ]
vector<double> Vehicle::GetVehicleBehind(int lane) {
  // Define the distance behind with the maximum safe distance
  double distance_behind = safe_s_dist_behind_ + vehicle_length_;

  // Get the predition vector: [ id, s,final_s, d, final_speed]
  auto preds = PredictSensorFusion(previous_path_x_.size());

  // Iterate over each predicted vehicle that is on the same given lane
  vector<double> vehicle_behind{};
  for (int i = 0; i < preds.size(); i++) {
    if (lane == CalculateLaneIndex(preds[i][3])) {
      if (preds[i][1] < s_ && preds[i][2] < end_path_s_ &&
          (end_path_s_ - preds[i][2]) < distance_behind) {
        // If a car was found behind and closer to the main vehicle, then update
        // the distance_behind and the vector vehicle_behind with this vehicle
        // information
        distance_behind = end_path_s_ - preds[i][2];
        vehicle_behind = {preds[i][4], distance_behind, preds[i][2]};
      }
    }
  }

  return vehicle_behind;
}

// Print information of state, speed, lane, trajectories and costs of the
// current update on the terminal
void Vehicle::PrintStatistics(vector<Trajectory> t, bool only_state_change) {
  if (state_ != state_prev_ || !only_state_change) {
    vector<string> state_str{"R", "KL", "LCL", "PLCL", "LCR", "PLCR"};
    std::cout << std::fixed << std::setprecision(2) << "State: " << std::setw(4)
              << state_str[state_prev_] << " -> " << std::setw(4)
              << state_str[state_] << "  Speed: " << speed_ << " -> "
              << trajectory_final_speed_ << " | " << projected_speed_ << std::fixed
              << std::setprecision(0) << "\tLane: " << lane_ << "->"
              << projected_lane_ << "|" << target_lane_;
    std::cout << "   " << std::fixed << std::setprecision(2);
    std::cout << "LaneSpeeds |";
    for (int i = 0; i < num_lanes_; i++) {
      std::cout << CalculateLaneSpeed(i) << "|";
    }
    std::cout << "\t";
    for (int i = 0; i < t.size(); i++) {
      std::cout << std::setw(4) << state_str[t[i].state] << "["
                << (t[i].cost < 0 ? 0 : t[i].cost) << "|" << std::fixed
                << std::setprecision(0) << t[i].projected_lane << ","
                << t[i].target_lane << "|" << std::fixed
                << std::setprecision(2);
      
      for (int j = 0; j < t[i].costs.size(); j++) {
        std::cout << (t[i].costs[j] < 0 ? 0 : t[i].costs[j])
                  << (j < t[i].costs.size() - 1 ? "," : "]");
      }
      if (i < t.size() - 1) {
        std::cout << "  ";
      }
    }
    std::cout << std::endl;
  }
}