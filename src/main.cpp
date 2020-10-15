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
#include <cstdio>
#include <fstream>
#include "logging.h"
#include "acado.h"
// for convenience
using json = nlohmann::json;

// global variable

bool flg_init = false;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }
#define MPH2MS 0.44704
// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s)
{
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos)
  {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos)
  {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x)
{
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++)
  {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order)
{
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++)
  {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++)
  {
    for (int i = 0; i < order; i++)
    {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main()
{
  using namespace std;
  uWS::Hub h;
  vector<vector<double>> control_output;
  // MPC is initialized here!
  MPC mpc;
  Log logSteering("../logging/LoggingSteering.json");
  h.onMessage([&mpc, &control_output, &logSteering](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {

    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2')
    {
      string s = hasData(sdata);
      if (s != "")
      {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry")
        {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

          // simulate latency compensation
          const double latency = 0.1;

          px = px + v * cos(psi) * latency;
          py = py + v * sin(psi) * latency;

          //converting to car's local coordinate system
          Eigen::VectorXd xvals(ptsx.size());
          Eigen::VectorXd yvals(ptsx.size());
          Eigen::MatrixXd translation(2, 2);
          translation << cos(-psi), -sin(-psi),
              sin(-psi), cos(-psi);
          Eigen::VectorXd pnt(2);
          Eigen::VectorXd local_pnt(2);

          for (int i = 0; i < ptsx.size(); i++)
          {
            // convert to vehicle coordinates
            pnt << ptsx[i] - px, ptsy[i] - py;
            local_pnt = translation * pnt;
            xvals[i] = local_pnt[0];
            yvals[i] = local_pnt[1];
            // std::cout <<"i: "<< i<< "lcl: " << local_pnt[0] <<", "<< local_pnt[1] << std::endl;
          }

          // add the 3rd order polynomial to the coeffecients
          auto coeffs = polyfit(xvals, yvals, 3);

          // Model steering and throttle using MPC
          double steer_value;
          double throttle_value;
          double cte;
          double epsi;

          cte = coeffs[0];
          epsi = -atan(coeffs[1]);

          Eigen::VectorXd state(6);
          state << 0, 0, 0, v, cte, epsi;

          auto vars = mpc.Solve(state, coeffs);
          steer_value = vars[0];
          throttle_value = vars[1];

          // acado setting
          vector<double> cur_state = {0, 0, v * MPH2MS, 0}; // because current pos is in local coordinate, x = y = psi = 0
          
          double ref_v = 20; // m/s
          if (flg_init == false)
          {
            printf("-------  initialized the acado ------- \n");
            control_output = init_acado();
            flg_init = true;
          }
          // printf("-------  previous_control_output ------- \n");
          // for (int i = 0; i < control_output[0].size(); i++)
          // {
          //   std::cout<< i <<" acceleration: "<< control_output[0][i] <<" steering: " <<control_output[1][i] <<endl;
          // }
          vector<double> predicted_states = motion_prediction(cur_state, control_output);
          vector<double> ref_states = calculate_ref_states(coeffs, ref_v);
          control_output = run_mpc_acado(predicted_states, ref_states, control_output);

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          //msgJson["steering_angle"] = steer_value / steer_normalizer * -1.0;
          // printf("steer value: %lf,   normalized value: %lf \n", steer_value, steer_value / deg2rad(25) * -1.0);
          msgJson["steering_angle"] = - control_output[1][0]; // steer_value / deg2rad(25) * -1.0;
          msgJson["throttle"] = control_output[0][0];// throttle_value;

          // Display the MPC predicted trajectory
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          // .. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          for (int i = 2; i < vars.size(); i = i + 2)
          {
            mpc_x_vals.push_back(vars[i]);
            mpc_y_vals.push_back(vars[i + 1]);
          }

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          // Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          for (int x = 0; x < 100; x = x + 5)
          {
            double y = polyeval(coeffs, x);
            next_x_vals.push_back(x);
            next_y_vals.push_back(y);
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          ///////////////////////////////////////////////////////////////////////////
          // logging msg
          float steering = {steer_value / deg2rad(25) * -1.0};
          logSteering.StartLogging(steering);
          // std::cout << msg << std::endl;
          ///////////////////////////////////////////////////////////////////////////

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
      }
      else
      {
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

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
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
