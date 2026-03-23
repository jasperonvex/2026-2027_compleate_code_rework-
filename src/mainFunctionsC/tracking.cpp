#include "mainFunctionsH\tracking.hpp"
#include "mainFunctionsH\functions.hpp"
#include "robot_setup.hpp"

double  vertDisDelta = 0;
double vertDis = 0;
double vertPrevDis = 0;

double horDisDelta = 0;
double horDis = 0;
double horPrevDis = 0;

double HeaderDelta = 0;
double Header = 0;
double Aheader = 0;
double HeaderPrev = 0;

double wheelDi = 2.0;

double wheelcric = wheelDi*M_PI;

double offsetX = 0;
double offsetY = 0;

int NumOfFails = 0;

double degToIn(double centiDeg){
    return ((centiDeg/100)/360) * wheelcric;
}


/*
 * ===============================================================
 * THE ODOM LOOP:
 * 
 * this part of the tracking system is the main function that is going to
 * running in the backgound, calculating the robots position based off the last position 
 * of the robot then will use the tracking wheels and inertial sensor finding the change 
 * in positon.
 * ===============================================================
 */

void tracking::odomLoop(){
    pros::delay(10);

    while(true){

        //this calculates the change in the vertical wheel sensor
        vertDis = degToIn(verticalWheel.get_position());
        vertDisDelta = vertDis - vertPrevDis;
        vertPrevDis = vertDis;

        //this calculates the change in the horisontle wheel sensor
        horDis = degToIn(horizontaleWheel.get_position());
        horDisDelta = horDis - horPrevDis;
        horPrevDis = horDis;

        //calculates the change in the inertial sensor and the average orintation
        Header = function.DegToRad(inert.get_heading());
        HeaderDelta = function.normalizeAngle(Header - HeaderPrev);
        Aheader = HeaderPrev + HeaderDelta/2;
        HeaderPrev = Header;

        //if the orinatation of the robot didnt change the offsets are as followed
        if(fabs(HeaderDelta) <= 1e-5){
            offsetX = horDisDelta;
            offsetY = vertDisDelta;
        }else{
            //if otherwise the offsets to the robots positon will be caculated in a arch
            offsetX = (2*(sin(Aheader))) * ((horDisDelta/HeaderDelta) + (horizontalOffset));
            offsetY = (2*(sin(Aheader))) * ((vertDisDelta/HeaderDelta) + (verticalOffset));
        }

        //  now we have to rotate the offsets based off of the avradge orintation
        // so we can caculate the offset of the robot based of f where it is going or facing

        //converting cartsain to polar
        float polarR = sqrt(pow(offsetX,2) + pow(offsetY,2));
        float polarT = atan2(offsetY,offsetX);

        //changing the oirntation
        polarT -= Aheader;

        //converting back to cartsain
        offsetX = polarR * cos(polarT);
        offsetY = polarR * sin(polarT);
        
        //finally we round it down to reduce the amount of noise created by the sensors
        offsetX = function.roundNearistThous(offsetX);
        offsetY = function.roundNearistThous(offsetY);

        if(isnan(offsetX) || isnan(offsetY) || isnan(Header)){
            // if thecaculations are resulting in a non number cancel it

            //counting the number of just incase I need to debug somthing 
            NumOfFails++;
            pros::lcd::set_text(0,"number of odom fails: " + std::to_string(NumOfFails));
        }else{
           //update the robots position 
            RoboPosition.x = getPositionData().x + offsetX;
            RoboPosition.y = getPositionData().y + offsetY;

            
        }
        //always update the header 
        RoboPosition.a = function.RadToDeg(Header);


        pros::delay(10);//needs to stay below 10ms
        
        
    }
}



/*
 *=====================================================================
 *GET POSITION DATA:
 *
 * This is the function that other main functions can use when they need
 * to get the robots position. The main reason why
 * this was created is to control the flow of data.
 * If I have the function it will release a snapshot of the robots 
 * position instead of a constantly updating variable which would cause a crash,
 * ====================================================================
 */

position tracking::getPositionData(){
    return RoboPosition;
}

/*
 *====================================================================
 * START ODOM LOOP:
 * 
 * This function starts the odom loop as a task allowing it to run in the background.
 * It also sets up all the sensors to be ready to calculate the robots position acuratly.
 * ====================================================================
 */

void tracking::startOdomLoop(){
    // resets the robots positon to zero
    RoboPosition = {0,0,0};

    verticalWheel.reset();
    horizontaleWheel.reset();
    inert.reset();

    //since the inertial sensor takes time to reset we want to
    //make sure it is done before we start the odom loop
    while (inert.is_calibrating()){
       pros::delay(10);
    }

    verticalWheel.set_data_rate(5);
    horizontaleWheel.set_data_rate(5);
    inert.set_data_rate(5);

    pros::delay(10);
    pros::Task odomLoopTask(odomLoop);
    

}

/*
 *====================================================================
 * THE SET POSITION:
 * 
 * this functions sets the positon of the robot. specifically used so
 * we can control where the robot starts
 *====================================================================
 */

void tracking::setPosition(double x, double y, double a){
    RoboPosition.x = x;
    RoboPosition.y = y;
    RoboPosition.a = a;
    inert.set_heading(a);
}



