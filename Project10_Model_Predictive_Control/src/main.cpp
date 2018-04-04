#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
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
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */

          vector<double> waypoints_car_coord_x;
          vector<double> waypoints_car_coord_y;

          for(size_t i = 0; i < ptsx.size(); i++){

              // Calculate distance of waypoint from car in global coordinates
              double shift_x = ptsx[i] - px;
              double shift_y = ptsy[i] - py;

              // Calculate distance in car coordinates since simulator takes data in car coordinates
              waypoints_car_coord_x.push_back(shift_x * cos(-psi) - shift_y * sin(-psi));
              waypoints_car_coord_y.push_back(shift_x * sin(-psi) + shift_y * cos(-psi));

          }

          double* ptrx = &waypoints_car_coord_x[0];
          double* ptry = &waypoints_car_coord_y[0];

          //Convert vector<double> to VectroXd
          Eigen::Map<Eigen::VectorXd> ptsx_transform(ptrx,6);
          Eigen::Map<Eigen::VectorXd> ptsy_transform(ptry,6);

          auto coeffs = polyfit(ptsx_transform, ptsy_transform, 3);

          //calculate cte
          double cte = polyeval(coeffs, 0); // 0 because car is at 0, 0

          //calculate epsi
          double epsi = -atan(coeffs[1]); // epsi is in car coordinates where initially psi is zero and x is zero in derivative of polynomial

          double steer_value = j[1]["steering_angle"];
          double throttle_value = j[1]["throttle"]; // NEED TO IMPLEMENT LATENCY HERE

          //latency in seconds
          const double latency_dt = 0.1;

          const double Lf = 2.67; //specific to car in the simulator

          //change in state variables during latency delay
          double x_after_delay = v * latency_dt;
          double y_after_delay = 0.0;
          double psi_after_delay = -v * steer_value / Lf * latency_dt;
          double v_after_delay = v + throttle_value * latency_dt;
          double cte_after_delay = cte + v * sin(epsi) * latency_dt;
          double epsi_after_delay = epsi - v * steer_value / Lf * latency_dt;

          Eigen::VectorXd state(6);
          state << x_after_delay, y_after_delay, psi_after_delay, v_after_delay, cte_after_delay, epsi_after_delay; // state variables in car coordinates

          auto vars = mpc.Solve(state, coeffs);

          //Display the waypoints/reference line as yellow line
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          double poly_inc = 2.5;
          int num_points = 25;
          for (int i = 1; i < num_points; i++){
              next_x_vals.push_back(poly_inc*i);
              next_y_vals.push_back(polyeval(coeffs, poly_inc*i));
          }

//          for (int i = 0; i < 100; i+=3){
//              next_x_vals.push_back(i);
//              next_y_vals.push_back(polyeval(coeffs, i));
//          }

          json msgJson;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          steer_value = vars[0];
          throttle_value = vars[1];

          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value/deg2rad(25); // normalize steering angle
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          for (size_t i = 2; i < vars.size(); i++){
              if(i%2 == 0){
                  mpc_x_vals.push_back(vars[i]);
              }
              else{
                  mpc_y_vals.push_back(vars[i]);
              }
          }

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
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
