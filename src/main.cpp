#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;

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
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); i++)
  {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x, y, map_x, map_y);
    if (dist < closestLen)
    {
      closestLen = dist;
      closestWaypoint = i;
    }

  }

  return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

  int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2( (map_y - y), (map_x - x) );

  double angle = abs(theta - heading);

  if (angle > pi() / 4)
  {
    closestWaypoint++;
  }

  return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
  int next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);

  int prev_wp;
  prev_wp = next_wp - 1;
  if (next_wp == 0)
  {
    prev_wp  = maps_x.size() - 1;
  }

  double n_x = maps_x[next_wp] - maps_x[prev_wp];
  double n_y = maps_y[next_wp] - maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
  double proj_x = proj_norm * n_x;
  double proj_y = proj_norm * n_y;

  double frenet_d = distance(x_x, x_y, proj_x, proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000 - maps_x[prev_wp];
  double center_y = 2000 - maps_y[prev_wp];
  double centerToPos = distance(center_x, center_y, x_x, x_y);
  double centerToRef = distance(center_x, center_y, proj_x, proj_y);

  if (centerToPos <= centerToRef)
  {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; i++)
  {
    frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
  }

  frenet_s += distance(0, 0, proj_x, proj_y);

  return {frenet_s, frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
  int prev_wp = -1;

  while (s > maps_s[prev_wp + 1] && (prev_wp < (int)(maps_s.size() - 1) ))
  {
    prev_wp++;
  }

  int wp2 = (prev_wp + 1) % maps_x.size();

  double heading = atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s - maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
  double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

  double perp_heading = heading - pi() / 2;

  double x = seg_x + d * cos(perp_heading);
  double y = seg_y + d * sin(perp_heading);

  return {x, y};

}

bool randomBool() {
  return rand() % 2 == 1;
}

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

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);
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

  //States in finite state machine
  typedef enum {
    Lane_Keep,
    Prep_LCL,
    Prep_LCR,
    LCL,
    LCR
  } State;
  //define two variables to keep track of lane numbers (3 in total) and refrence velocity i.e. 50 mph

  int lane = 1; //start off by choosing lane 1
  double ref_vel = 0.0;
  bool manoeuvre_safe = false;
  int cost = 100;
  State current_state;
  State next_state;
  current_state = Lane_Keep;
  bool best_lane_right = false;

  h.onMessage([&best_lane_right, &current_state, &next_state, &manoeuvre_safe, &cost, &ref_vel, &map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy, &lane](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
  uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;

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
          double car_yaw = j[1]["yaw"]; //car angle
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
          int prev_size = previous_path_x.size();
          //My TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds

          //get a refrence of car to find next_waypoint
            int wp_ref=-1;
            int car_ref_x = car_x;
            int car_ref_y = car_y;
            int car_ref_yaw = deg2rad(car_yaw);
            bool takeover = false;
            bool left_lane_dangerous = false;
            int closest_distance = 80000;
            std::vector<double> avg_vel ;


            if(prev_size > 0){
                car_s = end_path_s;
            }
            current_state = Lane_Keep;

          for (int i = 0; i < sensor_fusion.size(); i++)
          {
            int other_car_id = sensor_fusion[i][0];
            double other_car_x = sensor_fusion[i][1];
            double other_car_y = sensor_fusion[i][2];
            double other_car_vx = sensor_fusion[i][3];
            double other_car_vy = sensor_fusion[i][4];
            float other_car_s = sensor_fusion[i][5];
            float other_car_d = sensor_fusion[i][6];


            //if the car is in the same lane as our car then we
            //are interested to control car speed and distance
            if (other_car_d < (2 + 4 * lane + 2) && other_car_d > (2 + 4 * lane - 2) ) {
              //predict where the other car will be in the future
              double other_car_speed = sqrt(other_car_vx * other_car_vx + other_car_vy * other_car_vy);


              other_car_s += ((double) prev_size * 0.02 * other_car_speed);

              int distance_to_front_car = (other_car_s - car_s);

              //if we are approaching a car infront then do some action
              if ((other_car_s > car_s) && ((other_car_s - car_s) < 30) && distance_to_front_car < closest_distance)
              {
                closest_distance = distance_to_front_car;
                cout<<"closest distance to the front car" << closest_distance<<endl;



                if (closest_distance > 20) 
                {

                   ref_vel -= 0.113 ;
                   takeover = true;

                }
                else if(closest_distance < 15 )
                {

                   ref_vel -=0.287 ;
                   takeover = true;

                }else if(closest_distance<7){
                  takeover = false;
                }else{
                  ref_vel-=0.193;
                  takeover = true;
                }

              }

            }
          }

            if(ref_vel < 45.5){
                ref_vel+=0.123;
            }

          if(takeover && current_state == Lane_Keep){
              if (lane != 0) {
                  current_state = Prep_LCL;
              } else if (lane != 2) {
                  current_state = Prep_LCR;
              }
          }

          if(current_state == Prep_LCL){
              manoeuvre_safe = true;
              for (int i = 0; i < sensor_fusion.size(); i++){
                  //get other_car_d coordinates
                  float target_car_d = sensor_fusion[i][6];
                  if ((target_car_d < (2 + 4 * (lane - 1) + 2)) && (target_car_d > (2 + 4 * (lane - 1) - 2)) ) {
                      double other_car_vx = sensor_fusion[i][3];
                      double other_car_vy = sensor_fusion[i][4];
                      float other_car_s = sensor_fusion[i][5];
                      double other_car_speed = sqrt(other_car_vx * other_car_vx + other_car_vy * other_car_vy);
                      other_car_s += ((double) prev_size * 0.02 * other_car_speed);
                      avg_vel.push_back(other_car_speed*2.237);
                      double dist_s = other_car_s - car_s;
                      if(dist_s < 20 && dist_s > -20){
                          manoeuvre_safe = false;
                          left_lane_dangerous = true;
                          current_state = Lane_Keep;

                      }

                      if(left_lane_dangerous && lane != 2 && current_state == Lane_Keep){
                          current_state = Prep_LCR;
                      }

                      if(manoeuvre_safe ){
                         
                          if(car_s < other_car_s && ref_vel > other_car_speed*2.237){
                              takeover = false;
                              current_state = Lane_Keep;
                           }else{
                          current_state = LCL;
                          }
                      }



                  }

              }
          }

            if(current_state == Prep_LCR){
                manoeuvre_safe = true;
                for (int i = 0; i < sensor_fusion.size(); i++){
                    //get other_car_d coordinates
                    float target_car_d = sensor_fusion[i][6];
                    if ((target_car_d < (2 + 4 * (lane + 1) + 2)) && (target_car_d > (2 + 4 * (lane + 1) - 2)) ) {
                        double other_car_vx = sensor_fusion[i][3];
                        double other_car_vy = sensor_fusion[i][4];
                        float other_car_s = sensor_fusion[i][5];
                        double other_car_speed = sqrt(other_car_vx * other_car_vx + other_car_vy * other_car_vy);
                        other_car_s += ((double) prev_size * 0.02 * other_car_speed);
                        double dist_s = other_car_s - car_s;

                        if(dist_s < 20 && dist_s > -20){
                            manoeuvre_safe = false;
                            current_state = Lane_Keep;

                        }



                        if(manoeuvre_safe ){

                          if(car_s < other_car_s && ref_vel > other_car_speed*2.237){

                              takeover = false;
                              current_state = Lane_Keep;
                           }else{

                            current_state = LCR;
                            left_lane_dangerous = false;
                           }
                        }



                    }

                }
            }

            if(current_state == LCL){
                lane-=1;
                takeover = false;

            }

            if(current_state == LCR){
                lane+=1;
                takeover = false;
            }

            if(!takeover){
                current_state = Lane_Keep;
            }




          vector<double> ptsx; //for fitting spline into it
          vector<double> ptsy;

          double ref_x = car_x;
          double ref_y = car_y;
          double ref_angle = deg2rad(car_yaw);



          if (prev_size < 2) {

            double prev_carx = car_x - cos(car_yaw);
            double prev_cary = car_y - sin(car_yaw);

            ptsx.push_back(prev_carx);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_cary);
            ptsy.push_back(car_y);


          } else {

            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];

            double ref_x_prev = previous_path_x[prev_size - 2];
            double ref_y_prev = previous_path_y[prev_size - 2];
            ref_angle = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);


          }

          //lets create 3 spaced waypoints sothat we can fit a spline to it and then we augment more points from there

          std::vector<double> next_wp0 = getXY(car_s + 30, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          std::vector<double> next_wp1 =  getXY(car_s + 60, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);;
          std::vector<double> next_wp2 =  getXY(car_s + 90, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);;

          //push these points to our spaced waypoint collection

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          //shift car angle to be zero degrees
          for (int i = 0; i < ptsx.size(); i++)
          {
            //shift car angle to 0

            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;



            ptsx[i] = (shift_x * cos(0 - ref_angle) - shift_y * sin(0 - ref_angle));
            ptsy[i] = (shift_x * sin(0 - ref_angle) + shift_y * cos(0 - ref_angle));
          }
          // add 5 points that created to make spline
          tk::spline s;
          s.set_points(ptsx, ptsy);


          vector<double> next_x_vals;
          vector<double> next_y_vals;
          for (int i = 0; i < previous_path_x.size(); i++)
          {

            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          //augment points every 0.2 second to achieve desired speed
          double target_x = 30.0; //30 meter horizon
          double target_y =  s(target_x); //this will return a y value related to given x
          double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));

          double x_add_on = 0; // we start at origin of where the car is in car coordinate


          //augment rest of the path so car moves every 0.2 second
          for (int i = 1; i <= 50 - previous_path_x.size(); i++)
          {
            //formula as suggested by the walkthrough  d = N * .02 * velocity

            double N = (target_dist / (.02 * ref_vel / 2.24)); //2.24 is just a conversion from Mile PH to Meters PS
            double x_point = x_add_on + (target_x) / N;
            double y_point = s(x_point);

            x_add_on = x_point;  //update where our current x point is

            double x_ref = x_point;
            double y_ref = y_point;


            //now we need to transform local coordinate to global coordinate
            x_point = (x_ref * cos(ref_angle) - y_ref * sin(ref_angle));
            y_point = (x_ref * sin(ref_angle) + y_ref * cos(ref_angle));


            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }
          //END myTODO

          json msgJson;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
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
  h.onHttpRequest([](uWS::HttpResponse * res, uWS::HttpRequest req, char *data,
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
