#ifndef CAR_H
#define CAR_H
#include<vector>
using namespace std;

class Car
{
    public:

        Car(double lane_);


//
//        Car(vector<double> map_waypoints_x, vector<double> map_waypoints_y,
//                vector<double> map_waypoints_s, vector<double> map_waypoints_dx,
//                vector<double> map_waypoints_dy, int startpoint, double car_x, double car_y, double car_s);

       Car(vector<vector<double>> myfusion, int id,double car_s, double maximum_s );

        void SetID(int id_to_set);
        int  GetID();


        void SetSpeed(double speed);
        double GetSpeed();
        double MaxSpeedCalc(double current_speed);
        double SpeedTranslate(double x_speed, double y_speed);
        void SetD(int d_to_set, vector <double> d);
        vector<double> GetD(int d_to_get);

        void SetS(int  s_to_set, double s);
        double GetS(int s_to_get);


        void SetCoord(int coord_to_set, vector<double> coord);
        vector<double> GetCoord(int coord_to_get);
        void SetAdjustCoord_ForLane(int coord_to_set, vector<double>coord_,double lane_factor_, vector<double> d_);

        int LaneCalc(double traffic_d);
        void SetLane(int lane_to_set, int lane);
        int GetLane(int lane_to_get);

        void SetLaneFactor(int lane_to_set, double lane);
        double GetLaneFactor(int lane_to_get);

        void SetAdjustWaypoint_ForLane(int waypoint_to_set, vector<double>waypoint_,
                                       double lane_factor_, vector<double> d_);

        void SetDistFromMedian(int lane_to_set, double lane_factor_);
        double GetDistFromMedian(int lane_to_get);

        void SetDistToWaypoint(int point_to_set, double start_s_to_get, double end_s_to_get, int loopdelta, double maximum_s);
        double GetDistToWaypoint(int waypoint_to_get);



        void SetWaypoint(int waypoint_to_set, vector<double>waypoint);
        vector<double> GetWaypoint(int waypoint_to_get);
        map<double,double> GetWaypointMap();

        void SetDistanceAhead(double traffic_distance);
        double GetDistanceAhead();

        double CalcDistanceAhead(double traffic_s, double car_s, double maximum_s );

        void SetYaw(double yaw);
        double GetYaw();

        void SetPreviousCoord(vector <double> prior_coord);
        vector<double> GetPreviousCoord();


        virtual ~Car();

    protected:

    private:
        int id_;
        double speed_;
        double distance_ahead_;
        double yaw_;
        double prev_car_x_;
        double prev_car_y_;
        vector <double> x_coord_ {0,0,0,0};
        vector <double> y_coord_ {0,0,0,0};
        vector <double> s_ {0,0,0,0,0};
        vector <double> d_x_ {0,0,0,0};
        vector <double> d_y_ {0,0,0,0};

        vector <int> lane_ {0,0,0,0};
        vector <double> lane_factor_ {0,0,0,0};
        vector <double> dist_from_median_ {0,0,0,0};

        vector <double> waypoint_x_ {0,0,0,0};
        vector <double> waypoint_y_ {0,0,0,0};
        vector <double> dist_to_waypoint_ {0,0,0,0};

};

#endif // CAR_H
