#include <uWS/uWS.h>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// Definition of max values
#define MAX_JERK 10.0
#define MAX_SPEED 50.0
#define SAFE_S_DIST_AHEAD 30.0
#define MIN_S_DIST_AHEAD 30.0
enum State { R, LK, PLCL, LCL, PLCR, LCR };

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

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
  double ref_vel = 0.0;         // mph
  double ref_vel_prev = 0.0;    // mph
  double ref_dist_ahead = 0.0;  // m

  h.onMessage([&car_state, &lane, &ref_vel, &ref_vel_prev, &ref_dist_ahead,
               &map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
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

          // Calculate the size of the last path that the car was following
          int prev_size = previous_path_x.size();

          // Define the actual x and y points we will use for the planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // Finite state machine
          if (car_state == State::R) {
            vector<double> car_ahead{};
            for (int i = 0; i < sensor_fusion.size(); i++) {
              // Car is in my lane?
              float d = sensor_fusion[i][6];
              if (d < (2 + 4 * lane + 2) && d > (2 + 4 * lane - 2)) {
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double check_speed = sqrt(vx * vx + vy * vy);
                double check_car_s = sensor_fusion[i][5];

                // Check if there is a car ahead in the same lane
                if (check_car_s > end_path_s) {
                  car_ahead.push_back(check_speed);
                  car_ahead.push_back(check_car_s);
                  break;
                }
              }
            }

            // In case there isn't any car ahead or there is a distance greater
            // than the MIN_S_DIST_AHEAD, then change to Lane Keep State
            if (car_ahead.empty()) {
              car_state = State::LK;
              // return;
            } else {
              if (car_ahead[1] > 30.0) {
                car_state = State::LK;
              }
            }
          } else {
            if (car_state == State::LK) {
              // Find ref_v to use
              ref_vel = MAX_SPEED;
              for (int i = 0; i < sensor_fusion.size(); i++) {
                // Car is in my lane?
                float d = sensor_fusion[i][6];
                if (d < (2 + 4 * lane + 2) && d > (2 + 4 * lane - 2)) {
                  double vx = sensor_fusion[i][3];
                  double vy = sensor_fusion[i][4];
                  double check_speed = sqrt(vx * vx + vy * vy);
                  double check_car_s = sensor_fusion[i][5];

                  // Project s value outwards in time
                  check_car_s += ((double)prev_size * 0.02 * check_speed);

                  // Check s values greater than mine and s gap
                  if ((check_car_s > end_path_s) &&
                      ((check_car_s - end_path_s) < SAFE_S_DIST_AHEAD)) {
                    // too_close = true;
                    // TODO: set flag to change lanes, lower the speed
                    ref_vel = check_speed;
                    ref_dist_ahead = sensor_fusion[i][5];
                    // if (lane > 0) {
                    //   lane = 0;
                    // }
                  }
                }
              }
            }
            // if (prev_size > 0) {
            //   car_s = end_path_s;
            // }
            // Find ref_v to use
            // for (int i = 0; i < sensor_fusion.size(); i++) {
            //   // Car is in my lane?
            //   float d = sensor_fusion[i][6];
            //   if (d < (2 + 4 * lane + 2) && d > (2 + 4 * lane - 2)) {
            //     double vx = sensor_fusion[i][3];
            //     double vy = sensor_fusion[i][4];
            //     double check_speed = sqrt(vx * vx + vy * vy);
            //     double check_car_s = sensor_fusion[i][5];

            //     // Project s value outwards in time
            //     check_car_s += ((double)prev_size * 0.02 * check_speed);

            //     // Check s values greater than mine and s gap
            //     if ((check_car_s > end_path_s) &&
            //         ((check_car_s - end_path_s) < SAFE_S_DIST_AHEAD)) {
            //       // too_close = true;
            //       // TODO: set flag to change lanes, lower the speed
            //       ref_vel = check_speed;

            //       if (lane > 0) {
            //         lane = 0;
            //       }
            //     }
            //   }
            // }
            // bool too_close = false;

            // // TODO: Change velocities in the path planner
            // if (too_close) {
            //   ref_vel -= .224;
            // } else if (ref_vel < 49.5) {
            //   ref_vel += .224;
            // }

            // Create a list of widely spaced (x,y) waypoints, evenly spaced at
            // 30m
            vector<double> ptsx;
            vector<double> ptsy;

            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);

            // If previous size is almost empty, use the car as starting point
            if (prev_size < 2) {
              // Use two points that make the path tangent to the car
              double prev_car_x = car_x - cos(car_yaw);
              double prev_car_y = car_y - sin(car_yaw);

              ptsx.push_back(prev_car_x);
              ptsx.push_back(car_x);

              ptsy.push_back(prev_car_y);
              ptsy.push_back(car_y);
            }

            // Use the previous path's end points as starting reference
            else {
              // Redefine reference state as previous path end point
              ref_x = previous_path_x[prev_size - 1];
              ref_y = previous_path_y[prev_size - 1];

              double ref_x_prev = previous_path_x[prev_size - 2];
              double ref_y_prev = previous_path_y[prev_size - 2];
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
                getXY(car_s + 30, 2 + 4 * lane, map_waypoints_s,
                      map_waypoints_x, map_waypoints_y);
            vector<double> next_wp1 =
                getXY(car_s + 60, 2 + 4 * lane, map_waypoints_s,
                      map_waypoints_x, map_waypoints_y);
            vector<double> next_wp2 =
                getXY(car_s + 90, 2 + 4 * lane, map_waypoints_s,
                      map_waypoints_x, map_waypoints_y);

            ptsx.push_back(next_wp0[0]);
            ptsx.push_back(next_wp1[0]);
            ptsx.push_back(next_wp2[0]);

            ptsy.push_back(next_wp0[1]);
            ptsy.push_back(next_wp1[1]);
            ptsy.push_back(next_wp2[1]);

            for (int i = 0; i < ptsx.size(); i++) {
              // Shift car reference angle to 0 degrees
              double shift_x = ptsx[i] - ref_x;
              double shift_y = ptsy[i] - ref_y;

              ptsx[i] =
                  (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
              ptsy[i] =
                  (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
            }

            // Create a spline
            tk::spline s;

            // Set x and y points to the spline
            s.set_points(ptsx, ptsy);

            // Start with all of the previous path points from last time
            for (int i = 0; i < previous_path_x.size(); i++) {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }

            // Calculate how to break up spline points so that we travel at our
            // desired reference velocity
            double target_x = 30.0;
            double target_y = s(target_x);
            double target_dist =
                sqrt((target_x) * (target_x) + (target_y) * (target_y));

            double x_add_on = 0;

            // Adjust speed
            double err_vel = ref_vel - car_speed;

            if (fabs(err_vel / 2.24) > 9.75/10) {
              err_vel /= fabs(err_vel);
              err_vel *= (9.75 * 2.24)/10;
              std::cout << "err_vel= "<< err_vel <<" greater than 22: " << err_vel << std::endl;
            }

            double target_vel = car_speed + err_vel;

            for (int i = 1; i <= 50 - previous_path_x.size(); i++) {
              // 2.24 is to convert from miles/h to m/s
              double N = (target_dist / (0.02 * target_vel / 2.24));
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
          }
          json msgJson;

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

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