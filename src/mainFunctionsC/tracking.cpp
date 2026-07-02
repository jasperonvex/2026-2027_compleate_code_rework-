#include "mainFunctionsH/tracking.hpp"
#include "mainFunctionsH/functions.hpp"
#include "robot_setup.hpp"

tracking track;

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
        
        if(cameraTrack.TagPositionDetected)MergeCameraAndATLA;

        pros::delay(10);//needs to stay below 10ms
        
        
    }
}

void tracking::MergeCameraAndATLA(){
    RoboPosition.x += (RoboPosition.x - cameraTrack.getAtlaPosition().x) * cameraTrack.CameraTrust;
    RoboPosition.y += (RoboPosition.y - cameraTrack.getAtlaPosition().y) * cameraTrack.CameraTrust;
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
    pros::Task odomLoopTask([this]{
        this->odomLoop();
    });
    

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

// quadrant mutiplyers.
position quadrantOffset[5] = {
    {0,0,0},// a fillar so the count works
    {1,1,0},// I
    {1,-1,0},// II
    {-1,-1,0},// III
    {-1,1,0}// IV
};

/*
 *====================================================================
 * ABSOLUTE POSITIONING:
 * 
 * This distence tracking is used as a position reset. When I run the function 
 * I declare wich distence sensors I am usuing and with knowing wich quadrant Im
 * in and knowing what the robot's header I am able to use cos and sin to caculate
 * the x and Y offsets from the corner of the wall. After that I can then translate
 * that offset to the center of the feild giving me a global position of the robot 
 * based off of the wall, greatly booting acuracy and giving us leancy when setting
 * up the robot.
 * ====================================================================
 */


//0 = leftFront, 1= rightfront, 2 = right, 3 = left
void tracking::getAbsolutePosition(int dis1Num, int dis2Num){
    //updating every distence sensor
    distenceList[0].distence = function.MM_to_IN(Fl.get_distance());
    distenceList[1].distence = function.MM_to_IN(Fr.get_distance());
    distenceList[2].distence = function.MM_to_IN(R.get_distance());
    distenceList[3].distence = function.MM_to_IN(L.get_distance());

    int quadrant;

    //finding negetive or postivie  x y cords
    int ySing = getPositionData().y > 0 ? 1 : -1;
    int xSing = getPositionData().x > 0 ? 1 : -1;

    double robotAngle = getPositionData().a;

    // a row of possible if statments to figure out wich quadrant were in
    if(ySing == 1 && xSing == 1) quadrant = 1;
    if(ySing == -1 && xSing == 1) quadrant = 2;
    if(ySing == -1 && xSing == -1) quadrant = 3;
    if(ySing == 1 && xSing == -1) quadrant = 4;

    // caculating the x and y position by punching in the data to a function( its declared under this one )
    double DisXposition = calculateDisOffset(quadrant,distenceList[dis1Num].DisOffset.x,distenceList[dis1Num].DisOffset.y,distenceList[dis1Num].degreeOffset, robotAngle, distenceList[dis1Num].distence).x;
    double DisYposition = calculateDisOffset(quadrant,distenceList[dis2Num].DisOffset.x,distenceList[dis2Num].DisOffset.y,distenceList[dis2Num].degreeOffset, robotAngle, distenceList[dis2Num].distence).x;

    //roudning the positon to remove noise from the sensors 
    DisXposition = function.roundNearistThous(DisXposition);
    DisYposition = function.roundNearistThous(DisYposition);

    //setting the final position
    setPosition(DisXposition,DisYposition,robotAngle);


}

/*
 *====================================================================
 * CALCULATE DISTENCE OFFSET:
 * 
 * This is a child function that is used to slim down the main function.
 * this is where the main math happens where I caculate the robots global
 * positon off the wall.
 * ====================================================================
 */

position tracking::calculateDisOffset(int quad,double disOfsetX, double disOffsetY, double disDegOffset, double rA, double distence){
    double disXoffset = quadrantOffset[quad].x * (72 - abs(sin(function.DegToRad(function.normalizeDegAngle(rA + disDegOffset)))*distence));
    double disYoffset = quadrantOffset[quad].y * (72 - abs(sin(function.DegToRad(function.normalizeDegAngle(rA + disDegOffset)))*distence));

    double disOffRadianTh = atan2(disOfsetX,disOffsetY);
    double disOffsetRadianHyp = sqrt(pow(disOfsetX,2) + pow(disOffsetY,2));

    disOffRadianTh += function.DegToRad(rA);

    double OffsetX = sin(disOffRadianTh) * disOffsetRadianHyp;
    double OffsetY = cos(disOffRadianTh) * disOffsetRadianHyp;
    
    disXoffset -= OffsetX;
    disYoffset -= OffsetY;

    return {disXoffset,disYoffset};
}


/*
 *======================================================================
 *GET HEADER VIA DISTENCE SENSOR:
 *
 * This function gets the Gloabal orintation of the robot based off the wall
 * It dose this by usuing the two distences sensors on one side of the robot
 * in this scnerio the two sensors are on the front of the robot.
 * and then we assume we are facing towrds a specefic wall and with that
 * we can find the diffrence between the two values from each sensor to make a Right triangle against
 * the wall since we already know the adjecent line from the distence between the two sensors.
 * wich then can give us the theta of the of the triangle, wich can corrospond to the global orintation.
 * =====================================================================
 */


void tracking::getHeaderViaDis(double perpWallHead){
    double LFD = 0;
    double RFD = 0;

    // gets 10 values of the sensors and avradges them for acuracy
    for (size_t i = 0; i < 10; i++){
        
        LFD += function.MM_to_IN(Fl.get_distance());
        RFD += function.MM_to_IN(Fr.get_distance());
        pros::delay(1);
    }

    LFD = LFD/10;
    RFD = RFD/10;

    //caculates the theta of the right triangle from the diffrence between sensor 1 and two and how wide are they from each other.
    double theta = function.RadToDeg(atan2((LFD-RFD),abs(distenceList[1].DisOffset.x - distenceList[0].DisOffset.x)));

    //since we assume what wall we are, the theta acts as a offset so if we can subtract it from the perpwall header wich is what the
    // header would be if we were perfectly perpindiclar with it 
    double robotHeader = perpWallHead - theta;

    //a quick round to remove sound
    robotHeader = function.roundNearistThous(robotHeader);

    // a quick update to the position of the robots position
    inert.set_heading(robotHeader);

    position roboNewHeaderPos = getPositionData();

    roboNewHeaderPos.a = robotHeader;

    setPosition(roboNewHeaderPos.x,roboNewHeaderPos.y,roboNewHeaderPos.a);
    
}

void cameraTracking::Start_ATLA(){
    TagPositionDetected = false;

    vis.reset();
    vis.enable_detection_types(pros::AivisionModeType::tags);
    vis.set_tag_family(pros::AivisionTagFamily::tag_21H7);

    pros::Task startingATLA([]{
        track.cameraTrack.ATLA();
    });
   
}



void cameraTracking::ATLA(){
    double Fx = (Image_Width/2) / tan(Horosontle_FOV/2);
    double Fy = (Image_Height/2) / tan(Veritcal_FOV/2);

    

    while(true){
        

        auto objects = vis.get_all_objects();

        position roboPos = track.getPositionData();

        double Camx;
        double Camy;

        
        
        if(objects.empty()){
            pros::delay(50);
            camerastatus = cameraStatusOrignal + "No tag";
            TagPositionDetected = false;
            continue;
        
        }
        
        auto object = objects[0];
        
            if(!pros::AIVision::is_type(object, pros::AivisionDetectType::tag)){ 
                pros::delay(50);
                camerastatus = cameraStatusOrignal + "Not a tag ?";
                TagPositionDetected = false;
                continue;
            }

            TagID = object.id;

            double corn0[2] = {object.object.tag.x0,object.object.tag.y0};
            double corn1[2] = {object.object.tag.x1,object.object.tag.y1};
            double corn2[2] = {object.object.tag.x2,object.object.tag.y2};
            double corn3[2] = {object.object.tag.x3,object.object.tag.y3};

            TagPixlePosition[0] = (corn0[0]+corn1[0]+corn2[0]+corn3[0])/4;
            TagPixlePosition[1] = (corn0[1]+corn1[1]+corn2[1]+corn3[1])/4;
            
            double horizontalBearing = -atan(( TagPixlePosition[0] -(Image_Width/2) )/Fx);
            double verticalBearing = atan(((Image_Height/2) - TagPixlePosition[1])/Fy);
            
            pixleWidth = (function.GetDistence(corn0[0],corn0[1],corn1[0],corn1[1]) 
                                + function.GetDistence(corn3[0],corn3[1],corn2[0],corn2[1]))/2;

            if(pixleWidth < 10 || pixleWidth > 30){
                pros::delay(50);
                 camerastatus = cameraStatusOrignal + "Tag out of rang";
                 TagPositionDetected = false;
                //adds a limit so the focal distance dosent get too crazy
                continue;
               
            }
            
            double GroundDistence = (Focal_length_Pixels / pixleWidth) * cos(verticalBearing + cameraoffset.pitchOffset);

            double GlobalBearing = roboPos.a + horizontalBearing + function.DegToRad(cameraoffset.yawOffset);

            curentAprilTag = apriltags[TagID];

            double disFromRedAprilTags = function.GetDistence(curentAprilTag.TagPosition.x,curentAprilTag.TagPosition.y,roboPos.x,roboPos.y);
            double disFromBlueAprilTags = function.GetDistence(-curentAprilTag.TagPosition.x,-curentAprilTag.TagPosition.y,roboPos.x,roboPos.y);

            camerastatus = cameraStatusOrignal + "redSide & ";
            if(disFromBlueAprilTags < disFromRedAprilTags){
                curentAprilTag.TagPosition.x = -curentAprilTag.TagPosition.x;
                curentAprilTag.TagPosition.y = -curentAprilTag.TagPosition.y;
                camerastatus = cameraStatusOrignal + "blueSide & ";
            }

            int directFace = ((int)round(GlobalBearing / (M_PI / 2.0)) % 4 + 4) % 4;

            curentAprilTag.TagPosition.x += aprilTag_offset[directFace].x;
            curentAprilTag.TagPosition.y += aprilTag_offset[directFace].y;

            CamXOffset = (GroundDistence * sin(GlobalBearing));
            CamYoffset = (GroundDistence * cos(GlobalBearing));

            Camx = curentAprilTag.TagPosition.x - CamXOffset;
            Camy = curentAprilTag.TagPosition.y - CamYoffset;

            CroboX = Camx - (cameraoffset.Yoffset * sin(roboPos.a)) - (cameraoffset.xOffset * cos(roboPos.a));
            CroboY = Camy - (cameraoffset.Yoffset * cos(roboPos.a)) + (cameraoffset.xOffset * sin(roboPos.a));
        
           TagPositionDetected = true;
            /*
            4.65145 is the avrage error of the liner eqation to the recorded points
            55 is the number of points i recorded
            17.16473 is the avrage pixel width
            */
           double errorSigma = 4.65145 * sqrt(1.0 + (1.0 / 55.0) + (pow (pixleWidth - 17.16473,2) /1489.90897));

           CameraTrust = 100 * erf(tolerance / (errorSigma * sqrt(2)));
            
        camerastatus +=   "worked sucsufully";
        
        pros::delay(50);
    }
}


position cameraTracking::getAtlaPosition(){
    return {CroboX,CroboY,0};
}


