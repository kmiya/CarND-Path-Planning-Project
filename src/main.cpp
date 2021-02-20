#include <uWS/uWS.h>
#include <fstream>
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

bool isInLane(int target_lane, double current_d);
template<typename T>
int changeLane(int lane, double car_s, const T &sensor_fusion, const size_t &prev_size);
template<typename T>
int tryChangeLane(int lane, double car_s, const T &sensor_fusion, int change_lane, const size_t &prev_size);
bool isLaneChangeSafe(double car_s, const vector<double> &sense, const size_t &prev_size);

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
  // double max_s = 6945.554;

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

  int lane = 1;  // start in lane 1

  // reference velocity
  double ref_vel = 0.;  // mph
  const double max_speed = 49.5;  // mph

  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
                  &map_waypoints_dx, &map_waypoints_dy, &lane, &ref_vel, &max_speed]
                  (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                   uWS::OpCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (!s.empty()) {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Main car's localization Data
          const double car_x = j[1]["x"];
          const double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          const double car_d = j[1]["d"];
          const double car_yaw = j[1]["yaw"];
          // const double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          const auto previous_path_x = j[1]["previous_path_x"];
          const auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          const double end_path_s = j[1]["end_path_s"];
          // const double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          const auto sensor_fusion = j[1]["sensor_fusion"];

          const size_t prev_size = previous_path_x.size();

          // sensor fusion
          if (prev_size > 0) {
            car_s = end_path_s;
          }

          bool too_close = false;

          // find ref_v to use
          // sense: [id, x, y, vx, vy, s, d]
          for (const auto &sense : sensor_fusion) {
            // car is in my lane
            const float d = sense[6];
            if (isInLane(lane, d)) {
              const double vx = sense[3];
              const double vy = sense[4];
              const double check_speed = std::sqrt(vx * vx + vy * vy);
              double check_car_s = sense[5];

              // if using previous points can project s value out
              check_car_s += prev_size * 0.02 * check_speed;
              // check s values greater than mine and s gap
              const double safe_margin = 30.;
              const double dist_to_collision = check_car_s - car_s;
              if (check_car_s > car_s && dist_to_collision < safe_margin) {
                // lower reference velocity so we dont crash into the car in front of us,
                // could also flag to try to change lanes
                too_close = true;
              }
            }
          }
          if (too_close) {
            if (isInLane(lane, car_d)) {  // is my car trying to change lane?
              // check whether lane changing can be done
              // and do lane changing if it is safe
              lane = changeLane(lane, car_s, sensor_fusion, prev_size);
            }
          }
          // speed up and down slowly
          if (too_close) {
            // ref_vel -= .224 * (safe_margin - dist_to_collision) / safe_margin;
            ref_vel -= .224;
          } else if (ref_vel < max_speed) {
            ref_vel += .224;  // 5 meter per sec^2
          }

          json msgJson;
          /**
           *   Define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          vector<double> ptsx;
          vector<double> ptsy;
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // if previous size is almost empty, use the car as starting reference
          if (prev_size < 2) {
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          } else {  // use the previous path's end point as starting reference
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];
            double ref_x_prev = previous_path_x[prev_size - 2];
            double ref_y_prev = previous_path_y[prev_size - 2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }

          // In Frenet add evenly 30m spaced points ahead of the starting reference
          int ref_lane = 2 + 4 * lane;
          vector<double> next_wp0 = getXY(car_s + 30, ref_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 60, ref_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 90, ref_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          // transformation to the local car's coordinates
          for (size_t i = 0; i < ptsx.size(); ++i) {
            // shift car reference angle to 0 degree
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;
            ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
            ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
          }

          // create a spline
          tk::spline sp;

          // set (x, y) points to the spline
          sp.set_points(ptsx, ptsy);

          // Define the actual (x ,y) points we will use for the planner
          // start with all of the previous path points from last time
          vector<double> next_x_vals = previous_path_x;
          vector<double> next_y_vals = previous_path_y;

          // Calculate how to break up spline points so that we travel at our desired reference velocity
          const double target_x = 30.0; // meters far away from the car
          const double target_y = sp(target_x);
          const double target_dist = sqrt(target_x * target_x + target_y * target_y);

          double x_add_on = 0.;

          // Fill up the rest of our path planner after filling it with previous points,
          // here we will always output 50 points
          for (size_t i = 1; i <= 50 - prev_size; ++i) {
            const double ref_mps = 2.24;  // miles per second
            const double N = target_dist / (0.02 * ref_vel / ref_mps);
            double x_point = x_add_on + target_x / N;
            double y_point = sp(x_point);

            x_add_on = x_point;

            const double x_ref = x_point;
            const double y_ref = y_point;

            // rotate back to global coordinates after rotating it earlier
            x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
            y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

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
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER>, uWS::HttpRequest) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int,
                         char*, size_t) {
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
inline bool isInLane(const int target_lane, const double current_d) {
  return (current_d < 2. + 4 * target_lane + 2) && current_d > (2. + 4 * target_lane - 2);
}
template<typename T>
int changeLane(int lane, double car_s, const T &sensor_fusion, const size_t &prev_size) {
  if (lane > 0) {  // try changing lane to left
    const int change_lane = lane - 1;
    lane = tryChangeLane(lane, car_s, sensor_fusion, change_lane, prev_size);
    if (lane == change_lane)  // can be changed lane safely
      return lane;
  }
  if (lane < 2) {  // try changing lane to right
    const int change_lane = lane + 1;
    lane = tryChangeLane(lane, car_s, sensor_fusion, change_lane, prev_size);
  }
  return lane;
}
template<typename T>
int tryChangeLane(const int lane, const double car_s, const T &sensor_fusion,
                  const int change_lane, const size_t &prev_size) {
  for (const auto &sense: sensor_fusion) {
    const double sense_d = sense[6];
    if (!isInLane(change_lane, sense_d)) continue;
    if (!isLaneChangeSafe(car_s, sense, prev_size)) return lane;
  }
  return change_lane;  // safe to change lane
}
bool isLaneChangeSafe(const double car_s, const vector<double> &sense, const size_t &prev_size) {
  const double vx = sense[3];
  const double vy = sense[4];
  const double check_speed = std::sqrt(vx * vx + vy * vy);
  double check_car_s = sense[5];

  // if using previous points can project s value out
  check_car_s += prev_size * 0.02 * check_speed;
  const double safe_margin_ahead = 30.;   // meter
  const double safe_margin_behind = 15.;  // meter
  if (check_car_s >= car_s + safe_margin_ahead || check_car_s <= car_s - safe_margin_behind)
    return true;

  return false;
}