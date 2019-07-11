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
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  // Lane number
  int lane_number = 1;
  // Car speed (mph)
  double ref_speed = 0.0;
  // Total lanes
  int total_lanes = 3;

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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,
               &lane_number,&ref_speed,total_lanes]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
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

          int prev_size = previous_path_x.size();

          json msgJson;

          vector<double> ptsx;
          vector<double> ptsy;

          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          double prev_x = 0;
          double prev_y = 0;

          // Useful state variables
          bool too_close = false;
          bool should_change_lane = true;
          bool change_2_left = true;
          bool change_2_right = true;
          int target_lane = -1;

          if (prev_size > 0) {
            car_s = end_path_s;
          }

          int sf_vx = 3;
          int sf_vy = 4;
          int sf_s = 5;
          int sf_d = 6;
          int safe_dist = 20;

          // See if any car is in front of us and it's too close.
          for (int i=0; i<sensor_fusion.size(); ++i) {
            float d = sensor_fusion[i][sf_d];
            double vx = sensor_fusion[i][sf_vx];
            double vy = sensor_fusion[i][sf_vy];
            double speed = sqrt(vx*vx + vy*vy);
            double neighbor_s = sensor_fusion[i][sf_s];

            // Predict future distance of this neighboring car.
            neighbor_s += prev_size * 0.02 * speed;

            // There's a cars in our lane.
            if (d < (2+lane_number*4+2) && d > (2+lane_number*4-2)) {
              // The car is in front of us and soon its distance will be < 30m.
              if (neighbor_s > car_s && (neighbor_s - car_s) < 30) {
                too_close = true;
              }
            }

            // Always scan neighbor lanes to see if it's ok to change lane.
            if (lane_number == 0 || lane_number == total_lanes-1) {
              // Left-most lane, check the lane on the right.
              // Right-most lane, check the lane on the left.
              target_lane = (lane_number == 0)? 1: total_lanes - 2;
              if (d < (2+target_lane*4+2) && d > (2+target_lane*4-2)) {
                should_change_lane &= change_lane(neighbor_s, car_s, safe_dist);
              }
            } else {
              // Check the lane on the left.
              target_lane = lane_number - 1;
              if (d < (2+target_lane*4+2) && d > (2+target_lane*4-2)) {
                change_2_left &= change_lane(neighbor_s, car_s, safe_dist);
              }

              // Check the lane on the right.
              target_lane = lane_number + 1;
              if (d < (2+target_lane*4+2) && d > (2+target_lane*4-2)) {
                change_2_right &= change_lane(neighbor_s, car_s, safe_dist);
              }

              should_change_lane &= (change_2_left? change_2_left: change_2_right);
              target_lane = change_2_left? lane_number-1: lane_number+1;
            }
          }

          if (too_close) {
            // 2 options, change lane or break
            if (should_change_lane) {
              lane_number = target_lane;

              // Need to move fast when changing lane.
              ref_speed += ((ref_speed < 49.5)? 0.224: 0);
            } else {
              ref_speed -= 0.224;
            }
          } else if (ref_speed < 49.5) {
            ref_speed += 0.224;
          }

          if (prev_size < 2) {
            prev_x = ref_x - cos(ref_yaw);
            prev_y = ref_y - sin(ref_yaw);
          } else {
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];

            prev_x = previous_path_x[prev_size-2];
            prev_y = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y - prev_y, ref_x - prev_x);
          }

          ptsx.push_back(prev_x);
          ptsy.push_back(prev_y);
          ptsx.push_back(ref_x);
          ptsy.push_back(ref_y);

          // Generate 3 more ref way points which are 30, 60, 90m from vehicle.
          int diff = 30;

          for (int i=1; i<=3; ++i) {
            vector<double> wp = getXY(car_s + diff*i, (2 + 4*lane_number), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            ptsx.push_back(wp[0]);
            ptsy.push_back(wp[1]);
          }

          // Transform from global coord to car's coord.
          for (int i=0; i<ptsx.size(); ++i) {
            double local_x = ptsx[i] - ref_x;
            double local_y = ptsy[i] - ref_y;

            ptsx[i] = local_x * cos(0 - ref_yaw) - local_y * sin(0 - ref_yaw);
            ptsy[i] = local_x * sin(0 - ref_yaw) + local_y * cos(0 - ref_yaw);
          }

          // Declare spline
          tk::spline s;
          s.set_points(ptsx, ptsy);

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // Add previous path points
          for (int i=0; i<previous_path_x.size(); ++i) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // Definition of how to break spline to points.
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x*target_x+target_y*target_y);

          double accumulated_x = 0;

          // Fill the spline with points
          for (int i=1; i<=50-previous_path_x.size(); ++i) {
            double N = target_dist/(.02*ref_speed/2.24);
            double x_ = accumulated_x + target_x/N;
            double y_ = s(x_);
            accumulated_x = x_;

            double x_ref = x_;
            double y_ref = y_;

            // Transform back to global coord.
            x_ = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
            y_ = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

            x_ += ref_x;
            y_ += ref_y;

            next_x_vals.push_back(x_);
            next_y_vals.push_back(y_);
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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