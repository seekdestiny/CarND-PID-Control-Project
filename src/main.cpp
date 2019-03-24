#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  unsigned long long ob_cnt = 0; // number of elapsed timestamps

  // steering and throttle PID controllers
  PID pid_steer, pid_throttle;

  // How the parameters were tuned (manually + twiddle):
  // steering: no twiddle -- P=0.2, I=0.004, D=3.0; throttle: 0.3 fixed
  // steering: twiddle -- P=0.2, I=0.004, D=3.0; throttle: 0.3 fixed
  // steering: twiddle -- P=0.35, I=0.0188, D=10.0; throttle: 0.3 fixed
  // steering: no twiddle -- P=0.64, I=0.0238, D=14.0; throttle: twiddle -- P=2.0, I=0.0, D=3.0
  // steering: no twiddle -- P=0.64, I=0.0238, D=14.0; throttle: twiddle -- P=2.5, I=0.0, D=8.0
  // steering: twiddle -- P=0.35, I=0.0188, D=10.0; throttle: twiddle -- P=4, I=0.0, D=9.0
  // steering: no twiddle -- P=0.2, I=0.0188, D=7.0; throttle: no twiddle P=2.5, I=0.0, D=8.0
  // steering: no twiddle -- P=0.15, I=0.0188, D=5.0; throttle: no twiddle P=4.5, I=0.0, D=8.0
  // steering: no twiddle -- P=0.15, I=0.0188, D=5.0; throttle: no twiddle P=6.0, I=0.0, D=2.0

  pid_steer.Init(0.15, 0.0188, 5.0, 30, 0, 0.1, 0.005, 1.0, 0.2);
  pid_throttle.Init(6.0, 0.0, 2.0, 30, 0, 0.5, 0.005, 1.0, 0.1);
  
  h.onMessage([&pid_steer, &pid_throttle, &ob_cnt](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value, throttle_value;
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
         
          // update error and calculate steer_value at each step
          pid_steer.UpdateError(cte);
          steer_value = pid_steer.TotalError();

          pid_throttle.UpdateError(std::max(fabs(steer_value), fabs(cte)));
          throttle_value = 1.0 + pid_throttle.TotalError(); 
          if (throttle_value > 1.0) {
            throttle_value = 1.0;
          } else if (throttle_value <= 0.0) {
            throttle_value = 0.3;
          }

          // increment timestamp
          ++ob_cnt;

          // DEBUG
          std::cout
              << "\n======[" << ob_cnt << "]======"
              << "\nCross Track Error:               " << cte
              << "\nCurrent Speed:                   " << speed
              << "\nCurrent Steering Angle:          " << angle
              << "\nPID Corrected Steering:          " << steer_value
              << "\nPID Steering Coefficients:       " <<   "Kp = " << pid_steer.Kp
                                                       << ", Ki = " << pid_steer.Ki
                                                       << ", Kd = " << pid_steer.Kd
              << "\nPID Corrected Throttle:          " << throttle_value
              << "\nPID Throttle Coefficients:       " <<   "Kp = " << pid_throttle.Kp
                                                       << ", Ki = " << pid_throttle.Ki
                                                       << ", Kd = " << pid_throttle.Kd
              << "\n======[" << ob_cnt << "]======\n"
              << std::endl;

          // operate on vehicle
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
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
