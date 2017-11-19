#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <cmath>
#include <fstream>
#include <chrono>

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
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  // Load parameters from json file
  std::ostringstream param_buf; 
  std::ifstream param_file("../data/parameters.json"); 
  param_buf << param_file.rdbuf(); 
  auto param = json::parse(param_buf.str());
  
  // Initialize steering PID controller
  PID pid_steer;
  double init_Kp_steer = param["Steering"]["Kp"].get<double>();
  double init_Ki_steer = param["Steering"]["Ki"].get<double>();
  double init_Kd_steer = param["Steering"]["Kd"].get<double>();
  pid_steer.Init(init_Kp_steer, init_Ki_steer, init_Kd_steer);
  
  // Initialize speed PID controller
  PID pid_speed;
  double init_Kp_speed = param["Speed"]["Kp"].get<double>();
  double init_Ki_speed = param["Speed"]["Ki"].get<double>();
  double init_Kd_speed = param["Speed"]["Kd"].get<double>();
  pid_speed.Init(init_Kp_speed, init_Ki_speed, init_Kd_speed);
  
  double target_speed = param["Speed"]["Set"].get<double>();
  
  // Initialize data logger
	typedef std::chrono::milliseconds ms;
	
  std::ofstream datfile;
  datfile.open("../data/datalog.dat", std::ios_base::out);
  datfile << "time\tspeed\ttarget_speed\tcte\tangle\tsteer_value\n";
	datfile.close();
	
	ms tstart = std::chrono::duration_cast< ms >(std::chrono::system_clock::now().time_since_epoch());
  
  double steer_old = 0.;
  
  h.onMessage([&pid_steer, &pid_speed, &steer_old, &target_speed, &datfile, &tstart](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
		  
					// Steering controller
					pid_steer.UpdateError(cte);
					double steer_value = pid_steer.TotalError();
					
					// Speed controller
					double throttle;
					pid_speed.UpdateError(target_speed - speed);
					throttle = pid_speed.TotalError();
					
					// Data logger
					ms tnow = std::chrono::duration_cast< ms >(std::chrono::system_clock::now().time_since_epoch());
					ms tdiff = tnow - tstart;
					
					std::stringstream ss;
					ss << std::fixed << std::setprecision(3);
					ss << tdiff.count() << '\t'
						 << speed << '\t'
						 << target_speed << '\t'
						 << cte << '\t'
						 << angle << '\t'
						 << steer_value;
					
					// write data in stringstream to file
					std::ofstream datfile;
					datfile.open("../data/datalog.dat", std::ios_base::out | std::ios_base::app);
					datfile << ss.rdbuf() << std::endl;
					datfile.close(); 

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
