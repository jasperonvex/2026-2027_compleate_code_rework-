#include "mainFunctionsH\movements.hpp"
#include "mainFunctionsH\functions.hpp"
#include "mainFunctionsH\tracking.hpp"
#include "robot_setup.hpp"
#include "main.h"


/*
 *====================================================================
 * PURE PURSUIT:
 * 
 * The Pure Pursuit algorithm is the main path-following movment method we will use.
 * In the algorthim we take a seris of waypoints and insert new points to smoothen out the path.
 * After this we calculate the distence along the path along with the curvature, so we can 
 * give each point a target velicites within the accleration limits we set.
 * After the setting up section is compleat we run a loop where the robots position is updated 
 * and looks for the closet point of to the robot so then we can use that point to give us our 
 * target velocity and our look ahead distence. With the look ahead distence we calculate the
 * look ahead point wich is a set distence down the path so the robot will move towrds that point
 * The main way it dose this is right after we cacluate the curvature the robot needs to get to the
 * look ahead point and with tht we run the target velocity on a rate limiter, and send a 
 * individual target velocity to each wheel with a feed forward and feed back loop. The loop will
 * end when one of the two critera end loop varibles are met wether we are cut off by the time out
 * or we are with in a set distence away from the very last point.
 *====================================================================
 */

void Movement::PurePursuit(bool Isreverse, double timeout, std::vector<Waypoint> Waypoints){
    double timePassed;

    int toltP = 0;
    //this is section intersects points between the set waypoints.
    for (size_t i = 0; i < Waypoints.size() - 1; i++){
        
        // we get the distence and how many points can fit within the set disperpoint and then the angle.
        double distence = function.GetDistence(Waypoints[i].x, Waypoints[i].y, Waypoints[i+1].x, Waypoints[i+1].y);

        int NumOfPointInset = ceil(distence/disPerPoint);

        double angle = atan2(Waypoints[i+1].y - Waypoints[i].y, Waypoints[i+1].x - Waypoints[i].x);

        for (size_t g = 0; g < NumOfPointInset; i++){
            //with that weintersect points and yaking the data of the first waypoint till the last point
            path[toltP].x = ((g * disPerPoint) * cos(angle)) + Waypoints[i].x;
            path[toltP].y = ((g * disPerPoint) * cos(angle)) + Waypoints[i].y;
            path[toltP].targetVel = Waypoints[i].maxVeo;
            path[toltP].lookAhead = Waypoints[i].lookAhed;

            toltP++;
        }
        //puting the final point or the waypoint 
        path[toltP].x = Waypoints[i + 1].x;
        path[toltP].y = Waypoints[i + 1].y;
        path[toltP].targetVel = Waypoints[i + 1].maxVeo;
        path[toltP].lookAhead = Waypoints[i + 1].lookAhed;

        toltP++;
    }


    // now we calculate the distence the point is along the path
    double cumulativeDistence = 0;

    path[0].disAlongPoint = 0;

    for (size_t i = 0; i < toltP; i++){
        double segmentLength = function.GetDistence( path[i].x, path[i].y, path[i+1].x, path[i+1].y);
        cumulativeDistence += segmentLength;
        path[i].disAlongPoint = cumulativeDistence;
    }
    
    //and now the curvature of a point
    //this used the theroy that three randomly placed points not in a straigt
    //line can be connected by a circl thus having a curvature
    for (size_t i = 0; i < toltP; i++)
    {
        if(i == 0 || i == toltP -1) path[i].curvature = 0;
        else{
            double x1 = path[i-1].x;
            double y1 = path[i-1].y;
            double x2 = path[i].x + 0.0000001;//to reduce error
            double y2 = path[i].y;
            double x3 = path[i+1].x;
            double y3 = path[i+1].y;

            
            double k1 = 0.5 * (pow(x1,2) + pow(y1,2) - pow(x2,2) - pow(y2,2))/(x1 - x2);
            double k2 = (y1 - y2)/(x1 - x2);
            double b = 0.5 * (pow(x2,2) - 2 * x2 * k1 + pow(y2,2) - pow(x3,2) * k1 - pow(y3,2))/  (x3 * k2 - y3 + y2 - x2 * k2);
            
            double a = k1 - k2 * b;
            double R = sqrt(pow(x1 - a,2) + pow(y1 -b,2));
            double curvature = R != 0 ? 1/R : 0;

            if(isnan(curvature)) curvature = 0;//tho if the curvature is un calulatable then the line is a straight line

            path[i].curvature = curvature;
        }   
    }
            
    double K = 3;
    //this sets the target velocity lower if the curvature is higher.
    for (size_t i = 0; i < toltP; i++) {
        
        if(path[i].curvature != 0){
            path[i].targetVel = std::min(path[i].targetVel, K / path[i].curvature);
        }
    }

    //with this we assume the robot can only accerlate so fast
    //so we go from the last point to the first point
    //changing the velocity if it can only change so much
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
    //the driving loop
    while(true){
        //updates the robotis position
        position localPos = track.getPositionData();

        if(Isreverse) localPos.a = function.normalizeDegAngle(localPos.a + 180);

        //caclutes the closet point by calculating every points distence then if it is smaller then the last 
        //closest points distence then that will be the new closet point.
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

        //now we caclulate the look ahead point

        double lookAheadDis = path[closetP].lookAhead;
        double lookAheadPoint[2] = {path[prevLAP].x, path[prevLAP].y};

        int startIndex = std::max(prevLAP, closetP);
        int endIndex = toltP - 1;

        bool lookAheadIsFound = false;

       for (size_t i = 0; i < 2; i++){
            if(i > 0) lookAheadDis *= 1.35;
            for (size_t i = closetP; i < toltP; i++){
                //looks for a intersection to the path on the circle wich 
                //has the radius of the look ahead distence
                double dx = path[i+1].x - path[i].x;
            double dy = path[i+1].y - path[i].y;

            double fx = path[i].x - localPos.x;
            double fy = path[i].y - localPos.y;

            double a = dx * dx + dy * dy;
            double b = 2 * (fx * dx + fy * dy);
            double c = fx * fx + fy*fy - lookAheadDis*lookAheadDis;

            double discrim = pow(b,2) - 4*a*c;
            
            if (discrim < 0) continue;// if it is not zero we can continue

            discrim = sqrt(discrim);

            //there is two possible driminants so we calculate for both of them
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

       //finally if the look ahead point is not found then look 5 points ahead of the previous look ahead point.
       if(!lookAheadIsFound){
            int seed = std::min(startIndex + 5, endIndex);
            lookAheadPoint[0] = path[seed].x;
            lookAheadPoint[1] = path[seed].y;
            prevLAP = seed;
       }
       
       // now we calculate the x offset of the robot to the look ahead point
       double xOffset;
       

       double dx = lookAheadPoint[0] - localPos.x;
       double dy = lookAheadPoint[0] - localPos.y;
       
       double headingX = sin(function.DegToRad(localPos.a));
       double headingY = cos(function.DegToRad(localPos.a));

       double perpX = -headingY;
       double perpY = headingX;

       //if the robot is in revers we have to invert it.
       xOffset = Isreverse?(dx * perpX + dy * perpY) : -(dx * perpX + dy * perpY);

       // then we calculate the curvature to the look ahead point
       double curvatureA = 2*xOffset/(lookAheadDis*lookAheadDis);

       //this is the rate limiter so we accerlate smoothly into a target speed.
       double targetVelocity = std::clamp(path[closetP].targetVel - prevTargetVel, -maxRate, maxRate);
       prevTargetVel = targetVelocity;

       //we ace like each drivetrain has its own arch so we use the curvature to change the target velocity
       double leftTargetVel = (targetVelocity * (2 + curvatureA * trackWidth)/2)*60;
       double rightTargetVel = (targetVelocity * (2 - curvatureA * trackWidth)/2)*60;

       //left side feed forward and feed back loop
       double leftFF = Kv * leftTargetVel + Ka * ((leftTargetVel - prevLeftTS));//feed forward looks at what the speed should be
       double leftFB = Kp * (leftTargetVel - leftMg.get_voltage());//feed forward looks at the drivetrain and if its hitting the speed

       double leftVel = isnan(leftFF + leftFB) ? 0 : leftFF + leftFB;//if it dosent compute then set it to zero

       prevLeftTS = leftVel;

       //right feed forward and feed back
       double rightFF = Kv * rightTargetVel + Ka * ((rightTargetVel - prevRightTs));//feed forward looks at what the speed should be
       double rightFB = Kp * (rightTargetVel - rightMg.get_voltage());//feed forward looks at the drivetrain and if its hitting the speed

       double rightVel = isnan(rightFF + rightFB) ? 0 : rightFF + rightFB;//if it dosent compute then set it to zero

       prevRightTs = rightVel;

       //finally send the values to the drivetrain.
       if(Isreverse){
            leftMg.move_voltage(-leftVel);
            rightMg.move_voltage(-rightVel);
       }else{
            rightMg.move_voltage(rightVel);
            leftMg.move_voltage(leftVel);
       }


       //delay of the loop runs every 20ms
       pros::delay(20);
    
       //updates the end varibles
       double disFromEndPoint = function.GetDistence(localPos.x, localPos.y, path[toltP - 1].x, path[toltP - 1].y);
       
       timePassed += 20;

       //checks the end conditions 
       if(disFromEndPoint <= 4 || timePassed >- timeout){

        rightMg.brake();
        leftMg.brake();
        rightMg.move_voltage(0);
        leftMg.move_voltage(0);
        break;
       }



    }
}



