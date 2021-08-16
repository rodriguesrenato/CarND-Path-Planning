#include <uWS/uWS.h>

#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
// #include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include "trajectory.h"
#include "vehicle.h"

// Definition of max values
#define MAX_JERK 10.0
#define MAX_SPEED 50.0
#define SAFE_S_DIST_AHEAD 30.0
#define MIN_S_DIST_AHEAD 30.0

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  // Start in lane 1;
  int lane = 1;
  State car_state = State::R;

  // Have a reference velocity to target
  double ref_vel = 0.0;       // mph
  double ref_vel_prev = 0.0;  // mph
  double err_vel_prev = 0.0;
  double err_vel_sum = 0.0;
  double ref_dist_ahead = 0.0;  // m
  Vehicle ego_car;

  h.onMessage([&ego_car, &car_state, &err_vel_prev, &err_vel_sum, &lane,
               &ref_vel, &ref_vel_prev, &ref_dist_ahead, &map_waypoints_x,
               &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx,
               &map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data,
                                  size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];
          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          ego_car.SetLocalizationData(j[1]["x"], j[1]["y"], j[1]["s"],
                                      j[1]["d"], j[1]["yaw"], j[1]["speed"]);
          ego_car.SetPrevPath(j[1]["previous_path_x"], j[1]["previous_path_y"],
                              j[1]["end_path_s"], j[1]["end_path_d"]);
          ego_car.SetSensorFusion(j[1]["sensor_fusion"]);

          vector<State> possible_next_states = ego_car.GetSuccessorStates();

          vector<Trajectory> possible_ego_trajectories{};

          for (int i = 0; i < possible_next_states.size(); i++) {
            if (possible_next_states[i] = State::KL) {
              Trajectory kl_trajectory = ego_car.GenerateTrajectoryKL(
                  30.0, map_waypoints_s, map_waypoints_x, map_waypoints_y);
              possible_ego_trajectories.push_back(kl_trajectory);
            }
          }
          Trajectory ego_trajectory;

          if (possible_ego_trajectories.size() > 0) {
            ego_trajectory = possible_ego_trajectories[0];
            ego_car.SetChosenTrajectory(ego_trajectory);
          }

          // // Calculate the size of the last path that the car was following
          // int prev_size = previous_path_x.size();

          // // Define the actual x and y points we will use for the planner
          // vector<double> next_x_vals;
          // vector<double> next_y_vals;
          // auto a = next_x_vals;

          // // sensor_fusion =  [ id, x, y, vx, vy, s, d]: The vx, vy values
          // can
          // // be useful for predicting where the cars will be in the future.
          // For
          // // instance, if you were to assume that the tracked car kept moving
          // // along the road, then its future predicted Frenet s value will be
          // // its current s value plus its (transformed) total velocity (m/s)
          // // multiplied by the time elapsed into the future (s).

          // // Finite state machine
          // if (car_state == State::R) {
          //   vector<double> car_ahead{};
          //   for (int i = 0; i < sensor_fusion.size(); i++) {
          //     // Car is in my lane?
          //     float d = sensor_fusion[i][6];
          //     if (d < (2 + 4 * lane + 2) && d > (2 + 4 * lane - 2)) {
          //       double vx = sensor_fusion[i][3];
          //       double vy = sensor_fusion[i][4];
          //       double check_speed = sqrt(vx * vx + vy * vy);
          //       double check_car_s = sensor_fusion[i][5];

          //       // Check if there is a car ahead in the same lane
          //       if (check_car_s > end_path_s) {
          //         car_ahead.push_back(check_speed);
          //         car_ahead.push_back(check_car_s);
          //         break;
          //       }
          //     }
          //   }

          //   // In case there isn't any car ahead or there is a distance
          //   greater
          //   // than the MIN_S_DIST_AHEAD, then change to Lane Keep State
          //   if (car_ahead.empty()) {
          //     car_state = State::KL;
          //     // return;
          //   } else {
          //     if (car_ahead[1] > 30.0) {
          //       car_state = State::KL;
          //     }
          //   }
          // } else {
          //   if (car_state == State::KL) {
          //     // Find ref_v to use
          //     ref_vel = 50 / 2.24;
          //     ref_dist_ahead = 0.0;
          //     for (int i = 0; i < sensor_fusion.size(); i++) {
          //       // Car is in my lane?
          //       float d = sensor_fusion[i][6];
          //       if (d < (2 + 4 * lane + 2) && d > (2 + 4 * lane - 2)) {
          //         double vx = sensor_fusion[i][3];
          //         double vy = sensor_fusion[i][4];
          //         double check_speed = sqrt(vx * vx + vy * vy);
          //         double check_car_s = sensor_fusion[i][5];

          //         // Project s value outwards in time
          //         check_car_s += ((double)prev_size * 0.02 * check_speed);

          //         // Check s values greater than mine and s gap
          //         if ((check_car_s > end_path_s) &&
          //             ((check_car_s - end_path_s) < SAFE_S_DIST_AHEAD)) {
          //           // too_close = true;
          //           // TODO: set flag to change lanes, lower the speed
          //           ref_vel = check_speed;
          //           ref_dist_ahead = sensor_fusion[i][5];
          //           // if (lane > 0) {
          //           //   lane = 0;
          //           // }
          //         }
          //       }
          //     }
          //   }
          //   std::cout << "ego_s= " << j[1]["s"];
          //   for (int i = 0; i < sensor_fusion.size(); i++) {
          //     std::cout << " | " << (int)sensor_fusion[i][5] << "/"<<
          //     ((int)sensor_fusion[i][6])/4;
          //   }
          // ego_trajectory = ego_car.GenerateTrajectory(
          //     ref_vel, lane, 30, true, map_waypoints_s, map_waypoints_x,
          //     map_waypoints_y);
          // ego_car.SetChosenTrajectory(ego_trajectory);
          // }
          json msgJson;

          msgJson["next_x"] = ego_trajectory.x;
          msgJson["next_y"] = ego_trajectory.y;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  });  // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}