#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

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

  PID pid;
  bool twiddle = false;
  double p[3] = {0.05, 0.0001, 1.5};
  double dp[3] = {.01, .0001, .1};
  int n = 0;
  int max_n = 600;
  double total_cte = 0.0;
  double error = 0.0;
  double best_error = 10000.00;
  double tol = 0.001;
  int p_iterator = 0;
  int total_iterator = 0;
  int sub_move = 0;
  bool first = true;
  bool second = true;
  double best_p[3] = {p[0],p[1],p[2]};
  if(twiddle == true) {
    pid.Init(p[0],p[1],p[2]);
  }else {
    pid.Init(0.06, 0.00031, 1.29);
    //pid.Init(0.05, 0.0001, 1.5);
  }
  h.onMessage([&pid, &p, &dp, &n, &max_n, &tol, &error, &best_error, &p_iterator, &total_iterator, &total_cte, &first, &sub_move, &second, &twiddle, &best_p](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          double steer_value;
          double throttle_value = 0.3;
          json msgJson;
          
          if (twiddle == true){
            total_cte = total_cte + pow(cte,2);
            if(n==0){
              
              pid.Init(p[0],p[1],p[2]); 
            }
            //Steering value
            pid.UpdateError(cte);
            steer_value = pid.TotalError();
            
            // DEBUG
            //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << " Throttle Value: " << throttle_value << " Count: " << n << std::endl;
            n = n+1;
            if (n > max_n){ 
              
              
              //double sump = p[0]+p[1]+p[2];
              //std::cout << "sump: " << sump << " ";
              if(first == true) {
                std::cout << "Intermediate p[0] p[1] p[2]: " << p[0] << " " << p[1] << " " << p[2] << " ";
                p[p_iterator] += dp[p_iterator];
                //pid.Init(p[0], p[1], p[2]);
                first = false;
              }else{
                error = total_cte/max_n;
                
                if(error < best_error && second == true) {
                    best_error = error;
                    best_p[0] = p[0];
                    best_p[1] = p[1];
                    best_p[2] = p[2];
                    dp[p_iterator] *= 1.1;
                    sub_move += 1;
                    std::cout << "iteration: " << total_iterator << " ";
                    std::cout << "p_iterator: " << p_iterator << " ";
                    std::cout << "p[0] p[1] p[2]: " << p[0] << " " << p[1] << " " << p[2] << " ";
                    std::cout << "error: " << error << " ";
                    std::cout << "best_error: " << best_error << " ";
                    std::cout << "Best p[0] p[1] p[2]: " << best_p[0] << " " << best_p[1] << " " << best_p[2] << " ";
                }else{
                  //std::cout << "else: ";
                  if(second == true) {
                    std::cout << "Intermediate p[0] p[1] p[2]: " << p[0] << " " << p[1] << " " << p[2] << " ";
                    p[p_iterator] -= 2 * dp[p_iterator];
                    //pid.Init(p[0], p[1], p[2]);
                    second = false;
                  }else {
                    std::cout << "iteration: " << total_iterator << " ";
                    std::cout << "p_iterator: " << p_iterator << " ";
                    std::cout << "p[0] p[1] p[2]: " << p[0] << " " << p[1] << " " << p[2] << " ";
                    if(error < best_error) {
                        best_error = error;
                        best_p[0] = p[0];
                        best_p[1] = p[1];
                        best_p[2] = p[2];
                        dp[p_iterator] *= 1.1;
                        sub_move += 1;
                    }else {
                        p[p_iterator] += dp[p_iterator];
                        dp[p_iterator] *= 0.9;
                        sub_move += 1;
                    }
                    std::cout << "error: " << error << " ";
                    std::cout << "best_error: " << best_error << " ";
                    std::cout << "Best p[0] p[1] p[2]: " << best_p[0] << " " << best_p[1] << " " << best_p[2] << " ";
                  }
                }
                
              }
              

              if(sub_move > 0) {
                p_iterator = p_iterator+1;
                first = true;
                second = true;
                sub_move = 0;
              }
              if(p_iterator == 3) {
                p_iterator = 0;
              }
              total_cte = 0.0;
              n = 0;
              total_iterator = total_iterator+1;

              double sumdp = dp[0]+dp[1]+dp[2];
              if(sumdp < tol) {
                //pid.Init(p[0], p[1], p[2]);
                std::cout << "Best p[0] p[1] p[2]: " << best_p[0] << best_p[1] << best_p[2] << " ";
                //ws.close();
                //std::cout << "Disconnected" << std::endl;
              } else {
                std::string reset_msg = "42[\"reset\",{}]";
                ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
              }
              
            } else {
              msgJson["steering_angle"] = steer_value;
              msgJson["throttle"] = throttle_value;
              auto msg = "42[\"steer\"," + msgJson.dump() + "]";
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
           
          } else { //twiddle if
            pid.UpdateError(cte);
            steer_value = pid.TotalError();
        
            // DEBUG
            std::cout << "CTE: " << cte << " Steering Value: " << steer_value << " Throttle Value: " << throttle_value << " Count: " << n << std::endl;
            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = throttle_value;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            std::cout << msg << std::endl;
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          } //twiddle else
        }//telemtery
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
