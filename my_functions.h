#ifndef MY_FUNCTIONS_H_INCLUDED
#define MY_FUNCTIONS_H_INCLUDED
vector<int> MakelaneList(double lane_);
int LaneOpen(vector<Car> alltraffic , vector<double>lane_list, double s_ , int prev_size);

bool TooClose(vector<Car> alltraffic, int prev_size, double end_path_s, Car ego );



vector<int> CalcLaneTraffic(int lane_length, vector<Car> alltraffic, int lane_tested,double car_speed);
int CalcSafeTurn(vector<int> lane, double car_s);
void Openfile(vector<int> lane, double ego );
#endif // MY_FUNCTIONS_H_INCLUDED
