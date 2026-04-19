#include "mainFunctionsH\movements.hpp"
#include "mainFunctionsH\functions.hpp"
#include "mainFunctionsH\tracking.hpp"
#include "robot_setup.hpp"
#include "main.h"

void Movement::PurePuresuit(bool Isreverse, double timeout, std::vector<Waypoint> Waypoints){
    double timePassed;

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


    path[toltP - 1].targetVel = 0;

    for (size_t i = toltP - 2; i >= 0; i--){
        double d = path[i+1].disAlongPoint - path[i].disAlongPoint;
        double vi = path[i + 1].targetVel;
        double MaxPosbVel = sqrt(vi * vi + 2 * maxAccel * d);
        if(path[i].targetVel > MaxPosbVel){
            path[i].targetVel = MaxPosbVel;
        }
    }
    
    
    int prevCP = 0;
    int prevLAP = 0;

    double prevTargetVel = 0;

    double prevLeftTS = 0;
    double prevRightTs = 0;

    while(true){
        position localPos = track.getPositionData();

        if(Isreverse) localPos.a = function.normalizeDegAngle(localPos.a + 180);

        int closetP = prevCP;
        double closetPDis = function.GetDistence(localPos.x, localPos.y, path[closetP].x, path[closetP].y);
        if(prevCP != toltP){
            for (size_t i = prevCP; i < toltP-1; i++){
                double pointD = function.GetDistence(localPos.x, localPos.y, path[i].x, path[i].y);
                if(closetPDis > pointD){
                    closetPDis = pointD;
                    closetP = i;
                }
            }
            
        }

        prevCP = closetP;

        double lookAheadDis = path[closetP].lookAhead;
        double lookAheadPoint[2] = {path[prevLAP].x, path[prevLAP].y};

        int startIndex = std::max(prevLAP, closetP);
        int endIndex = toltP - 1;

        bool lookAheadIsFound = false;

       for (size_t i = 0; i < 2; i++){
            if(i > 0) lookAheadDis *= 1.35;
            for (size_t i = closetP; i < toltP; i++){
                double dx = path[i+1].x - path[i].x;
            double dy = path[i+1].y - path[i].y;

            double fx = path[i].x - localPos.x;
            double fy = path[i].y - localPos.y;

            double a = dx * dx + dy * dy;
            double b = 2 * (fx * dx + fy * dy);
            double c = fx * fx + fy*fy - lookAheadDis*lookAheadDis;

            double discrim = pow(b,2) - 4*a*c;
            
            if (discrim < 0) continue;

            discrim = sqrt(discrim);

            double t1 = (-b -discrim) / (2*a);
            double t2 = (-b + discrim) / (2*a);

            if(t1 >= 0 && t1 <= 1){

                lookAheadPoint[0] = path[i].x + t1 * dx;
                lookAheadPoint[1] = path[i].y + t1 * dy;
                lookAheadIsFound = true;
                prevLAP = i;
                break;
            }
            else if(t2 >= 0 && t2 <= 1){

                lookAheadPoint[0] = path[i].x + t2 * dx;
                lookAheadPoint[1] = path[i].y + t2 * dy;
                lookAheadIsFound = true;
                prevLAP = i;
                break;
            }
            }
            
       }

       if(!lookAheadIsFound){
            int seed = std::min(startIndex + 5, endIndex);
            lookAheadPoint[0] = path[seed].x;
            lookAheadPoint[1] = path[seed].y;
            prevLAP = seed;
       }
       
       double xOffset;
       

       double dx = lookAheadPoint[0] - localPos.x;
       double dy = lookAheadPoint[0] - localPos.y;
       
       double headingX = sin(function.DegToRad(localPos.a));
       double headingY = cos(function.DegToRad(localPos.a));

       double perpX = -headingY;
       double perpY = headingX;

       xOffset = Isreverse?(dx * perpX + dy * perpY) : -(dx * perpX + dy * perpY);

       double curvatureA = 2*xOffset/(lookAheadDis*lookAheadDis);


       double targetVelocity = std::clamp(path[closetP].targetVel - prevTargetVel, -maxRate, maxRate);
       prevTargetVel = targetVelocity;

       double leftTargetVel = (targetVelocity * (2 + curvatureA * trackWidth)/2)*60;
       double rightTargetVel = (targetVelocity * (2 - curvatureA * trackWidth)/2)*60;

       double leftFF = Kv * leftTargetVel + Ka * ((leftTargetVel - prevLeftTS));
       double leftFB = Kp * (leftTargetVel - leftMg.get_voltage());

       double leftVel = isnan(leftFF + leftFB) ? 0 : leftFF + leftFB;

       prevLeftTS = leftVel;

       double rightFF = Kv * rightTargetVel + Ka * ((rightTargetVel - prevRightTs));
       double rightFB = Kp * (rightTargetVel - rightMg.get_voltage());

       double rightVel = isnan(rightFF + rightFB) ? 0 : rightFF + rightFB;

       prevRightTs = rightVel;

       if(Isreverse){
            leftMg.move_voltage(-leftVel);
            rightMg.move_voltage(-rightVel);
       }else{
            rightMg.move_voltage(rightVel);
            leftMg.move_voltage(leftVel);
       }



       pros::delay(20);

       double disFromEndPoint = function.GetDistence(localPos.x, localPos.y, path[toltP - 1].x, path[toltP - 1].y);
       
       timePassed += 20;

       if(disFromEndPoint <= 4 || timePassed >- timeout){

        rightMg.brake();
        leftMg.brake();
        rightMg.move_voltage(0);
        leftMg.move_voltage(0);
        break;
       }



    }
}



