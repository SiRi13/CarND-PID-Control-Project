#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include "PID.hpp"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  } else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

void ResetSimulator(uWS::WebSocket<uWS::SERVER> ws) {
  std::string msg = "42[\"reset\"]";
  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}

int main() {
  uWS::Hub h;

  PID s_pid;
  PID t_pid;
  // Initialize the s_pid variable.
  // s_pid.Init(0.065, 6.513e-5, 2.08423);
  // s_pid.Init(0.0304563, 0.0, 2.08423);
  // s_pid.Init(0.03, 0.0, 2.0);
  s_pid.Init(0.0, 0.0, 0.0);
  // s_pid.Init(0.0933784, 0.0, 1.64645);
  t_pid.Init(0.2, 0.0, 3.0);

  unsigned n_msg = 0;
  h.onMessage([&s_pid, &t_pid, &n_msg](uWS::WebSocket<uWS::SERVER> ws,
                                       char *data, size_t length,
                                       uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value = 0.0, speed_value = 0.3;

          /*
           * Calcuate steering value here, remember the steering value is
           * [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed. Maybe
           * use
           * another PID controller to control the speed!
           */
          s_pid.UpdateError(cte);
          steer_value = s_pid.TotalError() / -deg2rad(25.0);
          ++n_msg;

          if ((s_pid.dp_[0] + s_pid.dp_[1] + s_pid.dp_[2] > s_pid.tolerance_)) {
            t_pid.UpdateError(30 - speed);
            speed_value = -1.0 * t_pid.TotalError();

            if (n_msg > 50 && (std::abs(cte) > 4.5 || speed < 0.5)) {
              s_pid.Twiddle();
              ResetSimulator(ws);
              s_pid.ResetErrors();
              n_msg = 0;
              return;
            } else if (n_msg > 100) {
              s_pid.Twiddle();
              n_msg = 0;
            }
            if (n_msg % 10 == 0) {
              std::cout << "Average Error: " << s_pid.Error()
                        << "\n Squared Error Sum: " << s_pid.squared_error_sum_
                        << "\n Best Error: " << s_pid.best_error_
                        << "\n Kp / Ki / Kd: " << s_pid.Kp << " / " << s_pid.Ki
                        << " / " << s_pid.Kd << "\n dp_: " << s_pid.dp_[0]
                        << " / " << s_pid.dp_[2] << " / " << s_pid.dp_[1]
                        << std::endl;
            }
          } else {
            t_pid.UpdateError(50 - speed);
            speed_value = -1.0 * t_pid.TotalError();
            speed_value *= (1.0 / (1.0 + abs(angle)));
          }

          // DEBUG
          /*
          std::cout << "#" << n_msg << " CTE: " << cte
                    << " Steering Value: " << steer_value
                    << " Speed Value: " << speed_value << std::endl;
*/
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = speed_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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
