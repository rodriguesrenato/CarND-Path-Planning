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

  // Read the map file
  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  // Populate the map waypoints vectors
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

  // Instantiate the main car object
  Vehicle main_car = Vehicle(50.0,0.5,3,0.02,4.0,30.0,10.0,2.0,50,5.0,3.0);
  main_car.SetCostWeights({1.1,1.0,100});

  h.onMessage([&main_car, &map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
               &map_waypoints_dx,
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
          // j[1] is the data JSON object received from simulator

          // Update the main car's localization Data
          main_car.UpdateLocalizationData(j[1]["x"], j[1]["y"], j[1]["s"],
                                       j[1]["d"], j[1]["yaw"], j[1]["speed"]);

          // Update the previous path data and the end s and d values of it
          main_car.UpdatePreviousPath(j[1]["previous_path_x"], j[1]["previous_path_y"],
                               j[1]["end_path_s"], j[1]["end_path_d"]);

          // Update the sensor fusion data
          // This is a list of all other cars on the same side of the road
          main_car.UpdateSensorFusion(j[1]["sensor_fusion"]);

          // Get the possible next states
          vector<State> possible_next_states = main_car.GetSuccessorStates();

          vector<Trajectory> possible_trajectories{};

          // Compute the trajectory of each possible state
          for (int i = 0; i < possible_next_states.size(); i++) {
            Trajectory trajectory = main_car.GenerateTrajectory(
                possible_next_states[i], map_waypoints_s, map_waypoints_x,
                map_waypoints_y);

            // If it returns a valid trajecotry, then calculate costs and append
            // this trajectory to the vector of possible trajectories
            if (trajectory.x.size() > 0) {
              main_car.CalculateCost(trajectory);
              possible_trajectories.push_back(trajectory);
            }
          }

          Trajectory best_trajectory;
          float check_cost = 0;
          if (possible_trajectories.size() > 0) {
            // Sort possible trajectories by cost
            std::sort(
                possible_trajectories.begin(), possible_trajectories.end(),
                [](Trajectory a, Trajectory b) { return a.cost < b.cost; });

            // Get the trajectory with the lowest cost
            best_trajectory = possible_trajectories[0];

            // Update main_car post params with the best trajectory
            main_car.SetChosenTrajectory(best_trajectory);
          }

          // Print statistics of the final main_car object
          main_car.PrintStatistics(possible_trajectories,false);

          // Build the JSON msg to be sent to the simulator
          json msgJson;

          msgJson["next_x"] = best_trajectory.x;
          msgJson["next_y"] = best_trajectory.y;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          // Send the control msg with the chosen trajectory to the simualtor
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