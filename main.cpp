#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <tuple>
#include <cstdio>
#include <cstdlib>

#include <algorithm>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "car.h"
#include "car.cpp"
#include "my_functions.h"
#include "my_functions.cpp"

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
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

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


    int lane = 1;// ego lane start
    double ref_val = 0.0;//MPH
    Car ego(lane); //set lane for Car object
    bool lane_change_in_process = false; //flag for lane chaneg
    //int lane_change_pause = 0;
  h.onMessage([&lane_change_in_process,&max_s, &ego, &ref_val, &lane,
 &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

          	//vector<double> next_x_vals;
          	//vector<double> next_y_vals;


          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          	//****************************************************************

          	//get unsued points

            // set car values as generated
            ego.SetCoord(0,{car_x,car_y});
            ego.SetS(0,car_s);
            ego.SetD(0,{car_d,0});
            ego.SetSpeed(car_speed);
            ego.SetYaw(car_yaw);

          	int prev_size = previous_path_x.size();
            int lane_move =-2;
            vector<vector<double>> myfusion = sensor_fusion;
            vector<Car> alltraffic;


            // create traffic objects
            for(int i = 0 ; i<myfusion.size(); i++){
                Car traffic(myfusion,i,ego.GetS(0),max_s);
                alltraffic.push_back(traffic);
            }
       bool too_close =  TooClose(alltraffic, prev_size, end_path_s, ego);
       vector<double> best_lane;
       int change_lane =-1;
       if((too_close == true) or ((lane_change_in_process == true) && (change_lane =-1))  ){
            vector<int> LaneList = MakelaneList(ego.GetLane(0));//get lanes that we can move into
           // cout<<"car is lane"<<ego.GetLane(0)<<endl;
           // cout<<"car  speed is "<<ego.GetSpeed()<<endl;
            for(int i = 0; i< LaneList.size();i++){
               // cout<<"lane: "<<LaneList[i]<<endl;
                vector<int> lane_with_traffic =  CalcLaneTraffic(6946, alltraffic, LaneList[i],ego.GetS(0));// maximum S = 6946, fill vector with traffic
                int safe_turn =  CalcSafeTurn(lane_with_traffic, ego.GetS(0)); //calculate the corridor of safety
             //   cout<<"score:" <<LaneList[i]<<endl;
                best_lane.push_back(safe_turn);
            }
           //returns the lane with biggest free space
            auto biggest = max_element(begin(best_lane), end(best_lane));
            auto lnum = distance(begin(best_lane), biggest); //get element to return corresponding lane number
           // cout << "Max element is " << *biggest
            //    << " at position " << distance(begin(best_lane), biggest) <<endl;

            if(*biggest > -1){
                change_lane = LaneList[lnum];
                lane_change_in_process = true;
            }
        }



        //adjust speed down if too close
             if(too_close == true){
                ref_val -= .224;
                }
        //asdjust speed down a  tad when changing lanes/
            if((too_close == false) && (lane_change_in_process == true)){

                if(ref_val >40){
                   ref_val -= .224;
                }
                if(ref_val <40){
                   ref_val += .224;
                }

             }

        //if not too close or changing lanes accelerate
            if((too_close == false) && (lane_change_in_process == false)){
                if(ref_val< 49.4){
                ref_val += .224;
                }
            }



//*******************part two end *****************************************************



            //point spread apart along the map
            vector<double> ptsx;
            vector<double> ptsy;

            //refernce x,y yaw states
            double ref_x = ego.GetCoord(0)[0];
            double ref_y = ego.GetCoord(0)[1];
            double ref_yaw = deg2rad(ego.GetYaw());


            //initialize points if empty
            if(prev_size<2){



            ego.SetPreviousCoord({ (ego.GetCoord(0)[0] - cos(ego.GetYaw())),
                                          (ego.GetCoord(0)[1] - sin(ego.GetYaw()))
                                        }) ;


            ptsx.push_back(ego.GetPreviousCoord()[0]); //make backward anchor point
            ptsx.push_back(ego.GetCoord(0)[0]); // current point

            ptsy.push_back(ego.GetPreviousCoord()[1]); //make backward anchor point
            ptsy.push_back(ego.GetCoord(0)[1]); // current point
            }

            //use the previous path end point as a starting reference

            else {

            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];

            //redefine reference state as previous path end point
            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);

            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);

            }


        //the Frenet ad evenly 45m spaced points ahead of starting reference.
        //used 45m because helped with jerk and acceleration issues
        if((change_lane == -1) ){
            double points_spaced_ahead =45;
            for(int i = 0; i<3;i++){
                ego.SetWaypoint(i,getXY(car_s + (points_spaced_ahead *(1+i)) ,(2+4*ego.GetLane(0)), map_waypoints_s, map_waypoints_x,map_waypoints_y));
                ptsx.push_back(ego.GetWaypoint(i)[0]);
                ptsy.push_back(ego.GetWaypoint(i)[1]);
            }

        } else {

            double points_spaced_ahead =45;
            double lane_factor = change_lane - ego.GetLane(0);
           // cout<<"got here"<<endl;
            int i =0;
            for(int i = 0; i<3;i++){
                ego.SetWaypoint(i,getXY(car_s + (points_spaced_ahead *(1+i)) ,(2+4*ego.GetLane(0)+(.33*lane_factor)), map_waypoints_s, map_waypoints_x,map_waypoints_y));
                ptsx.push_back(ego.GetWaypoint(i)[0]);
                ptsy.push_back(ego.GetWaypoint(i)[1]);

            }
            ego.SetLane(0,change_lane); //set car to new lane
            change_lane =-1;//reset flag
            lane_change_in_process = false;// reset lane change status
            }


        for (int i = 0; i<ptsx.size(); i++){

            //shift car reference angle to 0 degrees
            double shift_x = ptsx[i]-ref_x;
            double shift_y = ptsy[i]-ref_y;

            ptsx[i] = (shift_x *cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
            ptsy[i] = (shift_x *sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));

        }
            //create spline
            tk:: spline s;
            // give points to spline
            s.set_points(ptsx,ptsy);


           // define actual(x,y) points we will use with planner
            vector<double> next_x_vals;
            vector<double> next_y_vals;

            for (int i = 0; i< previous_path_x.size(); i++){
                    next_x_vals.push_back(previous_path_x[i]);
                    next_y_vals.push_back(previous_path_y[i]);
            }

            //calculate hot to break up points to travel at our desired speed.

            double target_x = 45.0;
            double target_y = s(target_x);// splines 45 meters ahead
            double target_dist = sqrt((target_x*target_x)+(target_y*target_y));

            double x_add_on = 0;

           // Fill out the path planner after filling it with previous points.  45 points

            for (int i = 1 ;i <= 45 - previous_path_x.size(); i++){
                double N = (target_dist/(.02 * ref_val/2.24));
                double x_point = x_add_on+(target_x)/N;
                double y_point = s(x_point);

                x_add_on = x_point; // steps x point by proper distance

                double x_ref = x_point;
                double y_ref = y_point;

                // back to global coordinates
               x_point = (x_ref *cos(ref_yaw)-y_ref*sin(ref_yaw));
               y_point = (x_ref *sin(ref_yaw)+y_ref*cos(ref_yaw));


                x_point += ref_x;
                y_point += ref_y;

                next_x_vals.push_back(x_point);
                next_y_vals.push_back(y_point);


            }














          	//**************************************************************************
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

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
















































































