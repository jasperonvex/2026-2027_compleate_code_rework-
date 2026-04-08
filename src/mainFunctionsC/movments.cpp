#include "mainFunctionsH\movements.hpp"
#include "mainFunctionsH\functions.hpp"

void Movement::PurePuresuit(bool Isreverse, double timeout, std::vector<Waypoint> Waypoints){

    int toltP = 0;

    for (size_t i = 0; i < Waypoints.size() - 1; i++){
        
        double distence = function.GetDistence(Waypoints[i].x, Waypoints[i].y, Waypoints[i+1].x, Waypoints[i+1].y);

        int NumOfPointInset = ceil(distence/disPerPoint);

        double angle = atan2(Waypoints[i+1].y - Waypoints[i].y, Waypoints[i+1].x - Waypoints[i].x);

        for (size_t g = 0; g < NumOfPointInset; i++){
            
            path[toltP].x = ((g * disPerPoint) * cos(angle)) + Waypoints[i].x;
            path[toltP].y = ((g * disPerPoint) * cos(angle)) + Waypoints[i].y;
            path[toltP].targetVel = Waypoints[i].maxVeo;
            path[toltP].lookAhead = Waypoints[i].lookAhed;

            toltP++;
        }
        
        path[toltP].x = Waypoints[i + 1].x;
        path[toltP].y = Waypoints[i + 1].y;
        path[toltP].targetVel = Waypoints[i + 1].maxVeo;
        path[toltP].lookAhead = Waypoints[i + 1].lookAhed;

        toltP++;
    }

    double cumulativeDistence = 0;

    path[0].disAlongPoint = 0;

    for (size_t i = 0; i < toltP; i++){
        double segmentLength = function.GetDistence( path[i].x, path[i].y, path[i+1].x, path[i+1].y);
        cumulativeDistence += segmentLength;
        path[i].disAlongPoint = cumulativeDistence;
    }
    
    for (size_t i = 0; i < toltP; i++)
    {
        if(i == 0 || i == toltP -1) path[i].curvature = 0;
        else{
            double x1 = path[i-1].x;
            double y1 = path[i-1].y;
            double x2 = path[i].x + 0.0000001;
            double y2 = path[i].y;
            double x3 = path[i+1].x;
            double y3 = path[i+1].y;


            double k1 = 0.5 * (pow(x1,2) + pow(y1,2) - pow(x2,2) - pow(y2,2))/(x1 - x2);
            double k2 = (y1 - y2)/(x1 - x2);
            double b = 0.5 * (pow(x2,2) - 2 * x2 * k1 + pow(y2,2) - pow(x3,2) * k1 - pow(y3,2))/  (x3 * k2 - y3 + y2 - x2 * k2);
            
            double a = k1 - k2 * b;
            double R = sqrt(pow(x1 - a,2) + pow(y1 -b,2));
            double curvature = R != 0 ? 1/R : 0;

            if(isnan(curvature)) curvature = 0;

            path[i].curvature = curvature;
        }   
    }
            
    double K = 3;

    for (size_t i = 0; i < toltP; i++) {
        
        if(path[i].curvature != 0){
            path[i].targetVel = std::min(path[i].targetVel, K / path[i].curvature);
        }
    }


    
    
}



