#include "car.h"
#include<vector>
#include<math.h>
#include<stdlib.h>
using namespace std;

Car::Car(double lane_){
SetLane(0,lane_);
}


Car::Car(vector<vector<double>> myfusion, int id,double car_s, double maximum_s){



        double x_coord =myfusion[id][1];
        double y_coord = myfusion[id][2];
        double x_velocity = myfusion[id][3];
        double y_velocity = myfusion[id][4];
        double s =  myfusion[id][5];
        double d =  myfusion[id][6];




        SetID(id);
        SetCoord(0,{x_coord,y_coord});
        SetS(0,s);
        SetD(0,{d,0});// 0 for y because it is not a coordinate d
        SetLane(0,LaneCalc(GetD(0)[0]));
        SetSpeed(SpeedTranslate(x_velocity,y_velocity));
        SetDistanceAhead(CalcDistanceAhead(GetS(0), car_s, maximum_s ));

//            cout << "ID: "          << GetID()
//                 << " x_coord: "    << GetCoord(0)[0]
//                 << " y_coord: "    << GetCoord(0)[1]
//                 << " Speed: "      << GetSpeed()
//                 << " s: "          << GetS(0)
//                 << " d: "          << GetD(0)[0]
//                 << " lane: "       << GetLane(0)
//                 << " Distance Ahead: "  << GetDistanceAhead()
//                 <<endl;

  //     cout<<"id "<< GetID()<<" Desetup"<<GetD(0)[0]<<endl;
}




Car::~Car()
{
    //dtor

}

void Car::SetID(int id_to_set){
id_= id_to_set;
}

int  Car::GetID(){
return id_;
}


void Car::SetSpeed(double speed){
    speed_ = speed;
}

double Car::GetSpeed(){
    return speed_;
}

double Car::SpeedTranslate(double x_speed, double y_speed){
    // translate x,y speed into velocity
    double traffic_speed = sqrt((x_speed* x_speed) + (y_speed *y_speed));
    return traffic_speed;
}








void Car::SetCoord(int coord_to_Set, vector<double> coord){
    x_coord_[coord_to_Set]= coord[0] ;
    y_coord_[coord_to_Set]= coord[1];
}
vector<double> Car::GetCoord(int coord_to_get){
    return {x_coord_[coord_to_get],y_coord_[coord_to_get]};
}

void Car::SetAdjustCoord_ForLane(int coord_to_set, vector<double>coord_,double lane_factor_, vector<double> d_){
    //dist_from_median_1_ = dist_from_median_1_+  lane_factor_1;
    x_coord_[coord_to_set] = coord_[0] + (lane_factor_ * d_[0]);
    y_coord_[coord_to_set] = coord_[1] + (lane_factor_ * d_[1]);
}




void Car::SetWaypoint(int waypoint_to_set, vector<double>waypoint){

    waypoint_x_[waypoint_to_set] = waypoint[0];
    waypoint_y_[waypoint_to_set] = waypoint[1];
}

vector<double> Car::GetWaypoint(int waypoint_to_get){
    return {waypoint_x_[waypoint_to_get], waypoint_y_[waypoint_to_get]};
}


map<double,double> Car::GetWaypointMap(){
    //create map to auto sort arrays together
    map<double,double> waypointmap;

    // create map with X as key and Y as value
    for(int  i = 0; i< waypoint_x_.size()-1;i++ ){
        waypointmap[waypoint_x_[i]]=waypoint_y_[i]; // add values to map

    }


return waypointmap;
}

void Car::SetAdjustWaypoint_ForLane(int waypoint_to_set, vector<double>waypoint_,double lane_factor_, vector<double> d_){
    //dist_from_median_1_ = dist_from_median_1_+  lane_factor_1;
    waypoint_x_[waypoint_to_set] = waypoint_[0] + (lane_factor_ * d_[0]);
    waypoint_y_[waypoint_to_set] = waypoint_[1] + (lane_factor_ * d_[1]);
}


void Car::SetD(int d_to_set, vector <double> d){
    d_x_[d_to_set] = d[0];
    d_y_[d_to_set] = d[1];
}
vector<double> Car::GetD(int d_to_get){
    return {d_x_[d_to_get],d_y_[d_to_get]};
}

void Car::SetS(int s_to_set, double s){
    s_[s_to_set] = s;
}
double Car::GetS(int s_to_get){
    return s_[s_to_get];
}

int Car::LaneCalc(double traffic_d){//take d and determines lane
    int lane_size = 4;// model lane size

    /*As car is in middle of the lane d is rounded up.
    The abs is used to remove any negative values.
    */
    double lane = abs(ceil((traffic_d/lane_size)))-1;
    //cout<<"d in: "<<traffic_d<<" lane out: " <<lane<<endl;

    return lane;
}




void Car::SetLane(int lane_to_set, int lane){
 lane_[lane_to_set] = lane;
}
int Car::GetLane(int lane_to_get){
 return lane_[lane_to_get];
}



void Car::SetLaneFactor(int lane_to_set, double lane){
 int lane_size = 4;
 int middle_of_lane = 2;
 lane_factor_[lane_to_set] = (lane_size * lane)- middle_of_lane;
}
double Car::GetLaneFactor(int lane_to_get){
return lane_factor_[lane_to_get];
}


void Car::SetDistToWaypoint(int point_to_set, double start_s_to_get, double end_s_to_get, int loopdelta, double maximum_s){
    double s_adjust=0;
    if (loopdelta<0){// adjust if car looped tack; tests if counter value is lower
        double s_adjust = maximum_s;
    }
    dist_to_waypoint_[point_to_set] = end_s_to_get - start_s_to_get + s_adjust;
}
double Car::GetDistToWaypoint(int waypoint_to_get){
    return dist_to_waypoint_[waypoint_to_get];
}


void Car::SetDistFromMedian(int lane_to_set, double lane_factor_){
    int lane_to_get = lane_to_set;
    dist_from_median_[lane_to_set] = Car::GetDistFromMedian(lane_to_get)+lane_factor_;
}

double Car::GetDistFromMedian(int lane_to_get){
    return dist_from_median_[lane_to_get] ;
}


double Car::MaxSpeedCalc(double current_speed){
  double acceleration_limit = 10*.02 *.80, // .90 so th speed does not error
        speed_limit = 22.35*.02 *.80,
        speed_limit_delta,
        speed;
    if(current_speed<speed_limit){ //checks to see if speed is less than limit
        speed_limit_delta = speed_limit - current_speed; // if less then calcs maximum accel
        if(speed_limit_delta> acceleration_limit){ //check to see if max a is larger than limit a
                speed = current_speed + acceleration_limit; //if yes sets speed using limit a
        } else {
                speed = current_speed + speed_limit_delta; //if no sets speed tom
        }
    } else {
        speed = speed_limit;// if speed is higher or equal to speed limit, speed is set to limit
    }
    return speed;
}

void Car::SetDistanceAhead(double traffic_distance){
    distance_ahead_ = traffic_distance;
}

double Car::GetDistanceAhead(){
    return distance_ahead_ ;
}





double Car::CalcDistanceAhead(double traffic_s, double car_s, double maximum_s ){
    //returns distance of traffic ahead
    double traffic_distance_ahead =0;
    if(car_s>traffic_s){//car is ahead of traffic
        //calculate distance behind
        double distance_behind = car_s-traffic_s;
        /*convert to distance ahead, (as s course is a loop) by
        adding in the loop size and subtracting the distance behind
        */
        traffic_distance_ahead = maximum_s - distance_behind;
    } else {
        // as traffic is ahead mathematically( traffic s > car s), calculate the distance ahead
        traffic_distance_ahead = traffic_s -car_s;
    }
return traffic_distance_ahead;
}



void Car::SetYaw(double yaw){
yaw_ = yaw;
}

double Car::GetYaw(){
return yaw_;
}


void Car::SetPreviousCoord(vector <double> prior_coord){
prev_car_x_ = prior_coord[0];
prev_car_y_ = prior_coord[1];
}

vector<double> Car::GetPreviousCoord(){
return{ prev_car_x_,prev_car_y_};
}


