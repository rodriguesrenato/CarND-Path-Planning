
#include <algorithm>
#include <iterator>
#include <map>
#include <string>
#include <vector>
#include <iostream>

#include "helpers.h"
#include "vehicle.h"
#include "spline.h"

using std::string;
using std::vector;

// Initialize Vehicle with configurations of feature limits and simulator
Vehicle::Vehicle(double speed_limit, double acc_limit, int num_lanes,
                 double time_step, double lane_width) {
  speed_limit_ = speed_limit;
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

// Convert d to lane index
int Vehicle::CalculateLaneIndex(double d) {
  if (d >= 0) {
    return d / lane_width_;
  } else {
    return d / lane_width_ - 1;
  }
}

vector<vector<double>> Vehicle::GenerateTrajectory(
    double target_speed, double target_lane, double target_traj_len, bool use_prev_path,
    vector<double> &map_waypoints_s, vector<double> &map_waypoints_x,
    vector<double> &map_waypoints_y) {
  // Define the actual x and y points we will use for the planner
  vector<double> next_x_vals{};
  vector<double> next_y_vals{};

  // Define the anchor points to create the spline
  vector<double> ptsx;
  vector<double> ptsy;

  double ref_x = x_;
  double ref_y = y_;
  double ref_yaw = deg2rad(yaw_);

  int prev_size = previous_path_x_.size();
  // If previous size is almost empty, use the car as starting point
  if (!use_prev_path || prev_size < 2) {
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
    ref_x = previous_path_x_[prev_size - 1];
    ref_y = previous_path_y_[prev_size - 1];

    double ref_x_prev = previous_path_x_[prev_size - 2];
    double ref_y_prev = previous_path_y_[prev_size - 2];
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
      getXY(s_ + 30, 2 + 4 * target_lane, map_waypoints_s, map_waypoints_x,
            map_waypoints_y);
  vector<double> next_wp1 =
      getXY(s_ + 60, 2 + 4 * target_lane, map_waypoints_s, map_waypoints_x,
            map_waypoints_y);
  vector<double> next_wp2 =
      getXY(s_ + 90, 2 + 4 * target_lane, map_waypoints_s, map_waypoints_x,
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

  if (use_prev_path) {
    // Start with all of the previous path points from last time
    for (int i = 0; i < previous_path_x_.size(); i++) {
      next_x_vals.push_back(previous_path_x_[i]);
      next_y_vals.push_back(previous_path_y_[i]);
    }
  }

  // Calculate how to break up spline points so that we travel at our
  // desired reference velocity
  double target_x = target_traj_len; // TODO: Refactor target_traj_len name
  double target_y = s(target_x);
  double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));


  // Adjust speed
  double kp = 0.1;
  double kd = 0.1;
  double ki = 0.5;
  double err_safe_s_dist = 0.0;

  double err_vel = target_speed - speed_;

//   if (ref_dist_ahead < SAFE_S_DIST_AHEAD && ref_dist_ahead > 0.0) {
//     err_safe_s_dist = SAFE_S_DIST_AHEAD - ref_dist_ahead;
//   }

//   double target_vel = speed_ + err_vel * kp + (err_vel - err_vel_prev) * kd -
//                       err_safe_s_dist * 0.1;

  std::cout << std::fixed << std::setprecision(5) << "err_vel= " << err_vel
            << "\tref_vel= " << target_speed << "\tcar_speed= " << speed_
            << "\tnext_x_vals_size= " << next_x_vals.size();
  // double err_acc = err_vel / (2.24 * 0.02);  // acc in m/s
  // if (fabs(err_acc) > 9.75) {
  //   err_acc /= fabs(err_acc);
  //   err_acc *= 9.75;
  //   std::cout << "*";
  // }
  // double target_vel = car_speed + err_acc *2.24* 0.02;
  // std::cout << err_acc << "\ttarget_vel= " << target_vel <<
  // std::endl;

  if (target_speed > speed_limit_) {
    target_speed = speed_limit_;
  }

  double x_add_on = 0;
  double spd = 0;

//   for (int i =1; i <= 50 - next_x_vals.size(); i++) {
//     // 2.24 is to convert from miles/h to m/s

//     // double N = (target_dist / (0.02 * target_speed));
//     // double x_point = x_add_on + (target_x) / N;

//     double x_point = 0.0 + speed_*i*time_step_ + acc_limit_*i*i*time_step_*time_step_/2.0 ;

//     double y_point = s(x_point);

//     x_add_on = x_point;

//     double x_ref = x_point;
//     double y_ref = y_point;

//     // Rotate back to normal after rotating it earlier
//     x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
//     y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

//     x_point += ref_x;
//     y_point += ref_y;

//     next_x_vals.push_back(x_point);
//     next_y_vals.push_back(y_point);
//   }  

// TODO: Fazer tudo em frenet, por ultimo converter para XY
  for (int i = next_x_vals.size(); i < 50; i++) {
    // 2.24 is to convert from miles/h to m/s

    double N = (target_dist / (0.02 * target_speed));
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
  std::cout << "\tnext_x_vals_size_final= " << next_x_vals.size() << std::endl;
  return vector<vector<double>>{next_x_vals,next_y_vals};
}