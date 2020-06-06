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

#define RISKMIN 0.6 //riskMax is 1
int lane = 1;
double ref_vel = 15.0;

int stFSM = READY;

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
               &map_waypoints_dx,&map_waypoints_dy]
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

          json msgJson;
          
          int prev_Size = previous_path_x.size();
          double riskLane0=0; //riskMax = 1
          double riskLane1=0;
          double riskLane2=0;
          double velLane0=27.7; //100kmph
          double velLane1=27.7;
          double velLane2=27.7;
          
          double frontCar_ds0_min=100;
          double frontCar_ds1_min=100;
          double frontCar_ds2_min=100;
          //collision avoidance using sensor fusion
          if(prev_Size>0)
          {
            car_s = end_path_s;
          }
          bool too_close = false;
          bool close = false;
          bool look4LC = false;
          for(int i =0; i<sensor_fusion.size();i++)
          {
            float d = sensor_fusion[i][6];
            if(d<(2+4*lane+2)&&d>(2+4*lane-2))
            {
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx+vy*vy);
              double check_car_s = sensor_fusion[i][5];
              check_car_s += ((double)prev_Size * 0.02 * check_speed); //future point of risk car
              if((check_car_s > car_s) && ((check_car_s - car_s) < 15))
              {
                too_close = true;
                //ref_vel = 29.5;
              }if((check_car_s > car_s) && ((check_car_s - car_s) < 30))
              {
                close = true;
                //ref_vel = 29.5;
              }
              if((check_car_s > car_s) && ((check_car_s - car_s) < 30))
              {
                look4LC = true;
                //ref_vel = 29.5;
              }
              
            }
            
            //data from lane 0
            if(d<(2+4*0+2)&&d>(2+4*0-2))
            {
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx+vy*vy);
              double check_car_s = sensor_fusion[i][5];
              check_car_s += ((double)prev_Size * 0.02 * check_speed); //future point of risk car
              if((check_car_s > car_s) &&(frontCar_ds0_min >  check_car_s-car_s))
              {
                frontCar_ds0_min = check_car_s-car_s;
                velLane0 = check_speed;
              }
              if( riskLane0 < riskOnLane(fabs(check_car_s-car_s)))
              {
                riskLane0 = riskOnLane(fabs(check_car_s-car_s));
              }
            }
            //data from lane 1
            if(d<(2+4*1+2)&&d>(2+4*1-2))
            {
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx+vy*vy);
              double check_car_s = sensor_fusion[i][5];
              check_car_s += ((double)prev_Size * 0.02 * check_speed); //future point of risk car
              if((check_car_s > car_s) &&(frontCar_ds1_min >  check_car_s-car_s))
              {
                frontCar_ds1_min = check_car_s-car_s;
                velLane1 = check_speed;
              }
              if( riskLane1 < riskOnLane(fabs(check_car_s-car_s)))
              {
                riskLane1 = riskOnLane(fabs(check_car_s-car_s));
              }
              
            }
            //data from lane 2
            if(d<(2+4*2+2)&&d>(2+4*2-2))
            {
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx+vy*vy);
              double check_car_s = sensor_fusion[i][5];
              check_car_s += ((double)prev_Size * 0.02 * check_speed); //future point of risk car
              if((check_car_s > car_s) &&(frontCar_ds2_min >  check_car_s-car_s))
              {
                frontCar_ds2_min = check_car_s-car_s;
                velLane2 = check_speed;
              }
              if( riskLane2 < riskOnLane(fabs(check_car_s-car_s)))
              {
                riskLane2 = riskOnLane(fabs(check_car_s-car_s));
              }
              
            }
          }
          
          if((too_close)&&(ref_vel > 0.0))
          {
            ref_vel -= 3.0*0.224;
          }
          else if((close)&&(ref_vel > 0.0))
          {
            ref_vel -= 1.0*0.224;
          }
          else if(ref_vel < 20.0)
          {
            ref_vel += 2.0*0.224;
          }
          else if(ref_vel < 49.0)
          {
            ref_vel += 1.0*0.224;
          }
          
          std::cout<<"lane :"<<lane<<" Risk : ("<< riskLane0<<" "<<riskLane1<<" "<<riskLane2<<" ) Vel : ("<<velLane0<<" "<<velLane1<<" "<<velLane2<<") " << " too_Clsor :"<<too_close;
          //Behaviour Planning - Finite State Machine
          /*
          double riskLine0 = riskOnLane(0, sensor_fusion); 
          double riskLine1 = riskOnLane(1, sensor_fusion);
          double riskLine2 = riskOnLane(2, sensor_fusion);
          double riskLeft = 1;
          double riskRight = 1;
          
          double velCurLane = velocityofCarinLane(lane, sensor_fusion);
          */
          double velCurLane, velRightLane, velLeftLane;
          static int ct=0;
          switch(stFSM)
          {
            case READY:
            {
              stFSM = KEEPLANE;
              break;
            }
            case KEEPLANE:
            {
              if(look4LC == true)
              {
                if(lane == 0)
                {
                  if(riskLane1<=RISKMIN)
                  {
                    stFSM = PLCR;
                  }
                }
                else if(lane == 2)
                {
                  if(riskLane1<=RISKMIN)
                  {
                    stFSM = PLCL;
                  }
                }
                else
                {
                  if(riskLane0 <= RISKMIN && riskLane2 <= RISKMIN)
                  {
                    if(riskLane0<=riskLane2)
                    {
                      stFSM = PLCL;
                    }
                    else
                    {
                      stFSM = PLCR;
                    }
                  }
                  else if(riskLane0 <= RISKMIN)
                  {
                    stFSM = PLCL;
                  }
                  else if(riskLane2 <= RISKMIN)
                  {
                    stFSM = PLCR;
                  }
                  else
                  {
                    stFSM = KEEPLANE;
                  }
                  
                }
                
              }
              break;
            }
            case PLCR:
            {
              velRightLane = (lane==0)?velLane1:velLane2;
              velCurLane = (lane==0)?velLane0:velLane1;
              if((velCurLane >= velRightLane) || (ref_vel<30.5))
              {
                stFSM = KEEPLANE;
              }
              else
              {
                stFSM = LCR;
              }
              break;
            }
            case PLCL:
            {
              velLeftLane = (lane==2)?velLane1:velLane0;
              velCurLane = (lane==2)?velLane2:velLane1;
              if((velCurLane >= velLeftLane)||(ref_vel<30.5))
              {
                stFSM = KEEPLANE;
              }
              else
              {
                stFSM = LCL;
              }
              break;
            }
            case LCR:
            {
              if(ct==0)
              	lane = lane + 1;
              ct++;
              if(ct == 10)
              {
                stFSM = KEEPLANE;
                ct = 0;
              }
              break;
            }
            case LCL:
            {
              if(ct==0)
              	lane = lane-1;
              ct++;
              if(ct == 10)
              {
                stFSM = KEEPLANE;
                ct = 0;
              }
              break;
            }
          default:
          {
            stFSM = READY;
          }
          
            
          }
          
          std::cout<<" stFSM "<<stFSM<<" look4LC" << look4LC;
          
          //generating pts required to define spline
          vector<double> ptsx;
          vector<double> ptsy;
          
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          
          if(prev_Size<2)
          {
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          else
          {
            ref_x = previous_path_x[prev_Size-1];
            ref_y = previous_path_y[prev_Size-1];
            double ref_x_prev = previous_path_x[prev_Size-2];
            double ref_y_prev = previous_path_y[prev_Size-2];
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }
          
          vector<double> next_waypoint_0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_waypoint_1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_waypoint_2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          ptsx.push_back(next_waypoint_0[0]);
          ptsx.push_back(next_waypoint_1[0]);
          ptsx.push_back(next_waypoint_2[0]);
          ptsy.push_back(next_waypoint_0[1]);
          ptsy.push_back(next_waypoint_1[1]);
          ptsy.push_back(next_waypoint_2[1]);
          //std::cout<<"spline point : ";
          for(int i = 0; i<ptsx.size(); i++)
          {
            double shift_x = ptsx[i]-ref_x;
            double shift_y = ptsy[i]-ref_y;
            ptsx[i] = shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw);
            ptsy[i] = shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw);
            //std::cout<<"(" << ptsx[i] << "," << ptsy[i]<< "), ";
          }
          
          
          tk::spline s;
          s.set_points(ptsx,ptsy);
          
          //trajactory generation - x,y points for path that car would visit
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          for(int i =0; i<previous_path_x.size(); i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          double target_x = 30; //planning 30m w.r.t x direction
          double target_y = s(target_x);
          double target_dist = sqrt(target_x*target_x+target_y*target_y);
          double N = target_dist/(0.02*ref_vel/2.24);
          
          double x_add_on = 0;
          //std::cout<<"target_dist= "<<target_dist<<"; N= "<<N;
          for(int i = 0; i< (50 - previous_path_x.size()); i++)
          {
            double x_point = x_add_on + (target_x)/N;
            double y_point = s(x_point);
            
            x_add_on = x_point;
            double x_ref = x_point;
            double y_ref = y_point;
            
            x_point = ref_x + (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
            y_point = ref_y + (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));
            
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
            
            //std::cout<<"(" << x_point << "," << y_point<< "," << N<< "," <<x_add_on << "), ";
          }
          //std::cout<<"x_add_on: " <<x_add_on ;
          std::cout<<std::endl;
          
          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */         


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