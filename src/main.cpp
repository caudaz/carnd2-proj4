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

  int iter = 0;
  int step = 0;

  float accu_error_squared = 0.0;
  float mean_error_squared = 0.0;
  
  float mean_error_squared_best = 0.0;
  float kp_best = 0.0;
  float ki_best = 0.0;
  float kd_best = 0.0;
  
int main()
{
  uWS::Hub h;


  
  PID pid;
  // TODO: Initialize the pid variable.
  if (iter == 0 && step ==0 ){pid.Init(0.00, 0.001, 0.00);std::cout << "hi0" << std::endl;}


  

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
		  if (step ==0 && iter > 0 ){pid.Init(pid.Kp + .05, pid.Ki, pid.Kd);}
		  if (step ==0 && iter > 0 && (iter % 50 == 0)){pid.Init(0.00, pid.Ki, pid.Kd + .05);}
/* 		  if (iter == 2 && step ==0 ){pid.Init(0.10, 0.001, 4.0);}	  
		  if (iter == 3 && step ==0 ){pid.Init(0.05, 0.001, 2.0);}
		  if (iter == 4 && step ==0 ){pid.Init(0.02, 0.001, 5.0);}
		  if (iter == 5 && step ==0 ){pid.Init(0.15, 0.001, 2.0);}
		  if (iter == 6 && step ==0 ){pid.Init(0.35, 0.001, 2.0);}
		  if (iter == 7 && step ==0 ){pid.Init(0.15, 0.001, 2.0);}
		  if (iter == 8 && step ==0 ){pid.Init(0.15, 0.001, 2.0);}   */


          // Steer PID		  
		  pid.UpdateError(cte);
		  steer_value = - pid.TotalError();
		  if (steer_value < -1.0) { steer_value = -1.0;}
		  if (steer_value > +1.0) { steer_value = +1.0;}

		  // UDACITY Throttle PID
		  const double target_speed = 35.0;
		  PID speed_pid;
		  speed_pid.Init(0.1, 0.0, 0.0);  
		  speed_pid.UpdateError(speed - target_speed);
		  double throttle_value  = - speed_pid.TotalError();
		  if     (throttle_value > +1.0){throttle_value = +1.0;}
		  else if(throttle_value < -1.0){throttle_value = -1.0;}		  

          if (step == 0){
			 std::cout << "*********** ITER=" << iter
			           << " ***********  PID Kp=" << pid.Kp << " PID Ki=" << pid.Ki << " PID Kd=" << pid.Kd << std::endl;	  
		  }

          // Error metric
		  accu_error_squared = accu_error_squared + pow(cte, 2.0);
		  mean_error_squared = accu_error_squared / (step + 1);
          
          // DEBUG
/*           std::cout << "Iter=" << iter 
		            << " Step=" << step 
					<< " CTE=" << cte 
					<< " StrVal=" << steer_value 
					<< " ThrVal=" << throttle_value 
					<< " AccumE=" << accu_error_squared 
					<< " MeanE=" <<  mean_error_squared 
					<<  std::endl; */

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value; // Default = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

		  step = step + 1;	
		  
          // reset to a new iteration if:
		  // step>1000 or outside track or slow speed (outside track most likely) 
		  if (step > 1600 || (fabs(cte)>3.0&&step>50) || (fabs(speed)<4.0 && step>50)){
			  std::string reset_msg = "42[\"reset\",{}]";
			  ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT); 
              step = 0;	
			  if (iter == 0){ 
			      mean_error_squared_best = mean_error_squared; 
				  kp_best = pid.Kp; ki_best = pid.Ki; kd_best = pid.Kd;
			  }
              if ((iter > 0) && ( mean_error_squared_best > mean_error_squared)){
				mean_error_squared_best = mean_error_squared;
				kp_best = pid.Kp; ki_best = pid.Ki; kd_best = pid.Kd;
			  }
              std::cout << "mean_error_squared_best=" << mean_error_squared_best
            			<< " kp_best=" << kp_best << " ki_best=" << ki_best << " kd_best=" << kd_best << std::endl;		
			  accu_error_squared = 0.0;
              iter = iter + 1;	
		  }
		  
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
