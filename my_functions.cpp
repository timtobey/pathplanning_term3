#include "my_functions.h"
#include <sstream>
#include <stdlib.h>
#include <numeric>
#include <cstdlib>
#define SSTR( x ) static_cast< std::ostringstream & >( \
        ( std::ostringstream() << std::dec << x ) ).str()



bool TooClose(vector<Car> alltraffic, int prev_size, double end_path_s, Car ego){

    double endpoint_s = ego.GetS(0); // get current s

    if(prev_size >0){
        endpoint_s = end_path_s; // if path generated get last s
    }

    bool too_close = false; //create variable to be output

    for (int i = 0; i<alltraffic.size(); i++){

        if(ego.GetLane(0) == alltraffic[i].GetLane(0)){ // test if traffic and lane is the same

            double  check_car_s = alltraffic[i].GetS(0); ;
            check_car_s +=  ((double)prev_size * .02 * alltraffic[i].GetSpeed()); //calculates end point with traffic seed

            if((check_car_s > endpoint_s) && ((check_car_s-endpoint_s)<30)){ // if distance is too small set true
           too_close = true;

            }

        }
    }

return too_close;

}


vector<int> MakelaneList(int lane_){
    vector<int> lane_list;
    switch(lane_){
        case 0:
            lane_list.push_back(1);
            break;
        case 1:
            lane_list.push_back(0);
            lane_list.push_back(2);
            break;

        case 2:
            lane_list.push_back(1);
            break;
    }
return lane_list;

}

int LaneOpen(vector<Car> alltraffic , vector<double>lane_list, double s_ , int prev_size){
    int open_lane = -1;
    for(int j = 0; j<lane_list.size(); j++){

            for (int i = 0; i<alltraffic.size(); i++){

                if (lane_list[j] == alltraffic[i].GetLane(0)){

                    double  check_car_s = alltraffic[i].GetS(0); ;
                    check_car_s +=  ((double)prev_size * .02 * alltraffic[i].GetSpeed());

                    if((check_car_s > s_) && ((check_car_s- s_)<30)){
                        open_lane = lane_list[j];
                    }
                }

            }
    }
return open_lane;
}


vector<int> CalcLaneTraffic(int lane_length, vector<Car> alltraffic, int lane_tested,double car_speed){

    int points =30; // number of points to calc forward
    int buffer = 5; // so calc does not extned beyond vector end
    vector<int> lane((lane_length+points+buffer),1);//creates vector ones the length of the lane
    int tsize = alltraffic.size();
    double distance_traveled = 0;
    for(int i=0;i<tsize; i++){ //iterate through the traffic
            if(alltraffic[i].GetLane(0) ==  lane_tested ){ //check to make sure traffic is in same lane as being tested

                if(alltraffic[i].GetSpeed()>= car_speed){
                     distance_traveled = round(alltraffic[i].GetSpeed()*points*.02); //mps adjusted??
                } else{

                 distance_traveled = 10; // if traffic is slower than car then only close five boxes for safety
                }



//                cout<<"traffic speed: "<<alltraffic[i].GetSpeed()
//                <<", distance_traveled: "<<distance_traveled
//               <<endl;


                //range: starts where traffic is currently to where it will end up
                // changes this vector range to zero
                for(int j= round(alltraffic[i].GetS(0)); j< (alltraffic[i].GetS(0) + distance_traveled); j++){
                    lane[j]  = 0;
                   // cout<<"lane value"<<lane[j]<<"for item"<<j<<endl;
                }
            }
    }


    return lane;

}


int CalcSafeTurn(vector<int> lane, double car_s){

    int safe_begin = car_s -5 ;// car position less safety for traffic close behind
    int safe_corridor  = 25;// amount of distance needed to safely change lanes
    bool cooridor = true;

    int freespace = 0;// amount of freespace to change lanes into
    int i = safe_begin; // where the turning corridor begins
    while(cooridor){
        //cout<<"lane state"<<lane[i]<<endl;
        switch(lane[i])
        {
            case 0: // zero means there is traffic there so cant go there
                cooridor = false;
                break;
            case 1:
              freespace= freespace + 1;// this means not traffic in this spot
              if(i== lane.size()){
                cooridor = false;// this is here so I does not go beyond the size of the array
              } else{
                i++;}//test next spot in the array for traffic
                break;
        }
    }


    //cout<<"coordior size "<<freespace<<endl;
    if (freespace < safe_corridor){//returns a value that reflects the length of freespace in the lane
        freespace = -1;//if the size is less than the required safety corridor a -1 is returned
    }

//Openfile(lane,car_s ); diagnostics


return freespace;
}



void Openfile(vector<int> lane, double ego ){
    ofstream mydata;
    mydata.open ("data2.txt");
   int tsize = lane.size();
    string  dataline = SSTR(ego)+"\n";
     mydata <<dataline;
    for(int i=0;i<tsize; i++){
      //string  dataline = SSTR(lane[i])+","+SSTR(ego[i])+"\n";
       string  dataline = SSTR(lane[i])+"\n";
        mydata <<dataline;
    }
    mydata.close();
    cout<<"data file closed"<<endl;
}



