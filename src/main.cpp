#include <uWS/uWS.h>

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
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
          ego_car.SetLocalizationData(j[1]["x"], j[1]["y"], j[1]["s"],
                                      j[1]["d"], j[1]["yaw"], j[1]["speed"]);
          // // Previous path data given to the Planner and path's end s and d
          // values
          ego_car.SetPrevPath(j[1]["previous_path_x"], j[1]["previous_path_y"],
                              j[1]["end_path_s"], j[1]["end_path_d"]);
          // Sensor Fusion Data, a list of all other cars on the same side of
          // the road.
          ego_car.SetSensorFusion(j[1]["sensor_fusion"]);

          vector<State> possible_next_states = ego_car.GetSuccessorStates();

          vector<Trajectory> possible_ego_trajectories{};

          for (int i = 0; i < possible_next_states.size(); i++) {
            Trajectory trajectory = ego_car.GenerateTrajectory(
                possible_next_states[i], map_waypoints_s, map_waypoints_x,
                map_waypoints_y);
            if (trajectory.x.size() > 0) {
              ego_car.CalculateCost(trajectory);
              possible_ego_trajectories.push_back(trajectory);
            }
          }
          Trajectory ego_trajectory;
          // Sort min cost
          if (possible_ego_trajectories.size() > 0) {
            std::sort(
                possible_ego_trajectories.begin(),
                possible_ego_trajectories.end(),
                [](Trajectory a, Trajectory b) { return a.cost < b.cost; });
                vector<string> state_str{"R", "KL", "PLCL", "LCL", "PLCR", "LCR"};
            for (int i = 0; i < possible_ego_trajectories.size(); i++) {
              std::cout << state_str[possible_ego_trajectories[i].state] << "[" << possible_ego_trajectories[i].cost << "] ";
            }
            ego_trajectory = possible_ego_trajectories[0];
            ego_car.SetChosenTrajectory(ego_trajectory);
          }
          ego_car.PrintStatistics();

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