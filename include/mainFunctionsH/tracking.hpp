 #include "string"
struct position{
   double x;
   double y;
   double a;
};

struct distenceData{
    double distence;
    position DisOffset;
    double degreeOffset;
};

struct aprilTag
{
    int id;
    position TagPosition;
    
};


class cameraTracking{
    private:

   struct cameraOffset {
        double xOffset;
        double Yoffset;
        double yawOffset;
        double pitchOffset;
        double  angleOffset;
   }
    cameraoffset = {0,5.375,5.65,0};

    double Image_Width = 320;
    double Image_Height = 240;

    double Horosontle_FOV = 74;
    double Veritcal_FOV = 63;

    

    double Focal_length_Pixels = 157;// (distence * pixlewidth) / actual width
    
    aprilTag apriltags[5] = {
        {0,{0,0}},
        {1,{-48,24}},
        {2,{-48,-24}},
        {3,{-24,-48}},
        {4,{24,-48}}
    };

    //distance from tag to center of the goal is 1.756
    position aprilTag_offset[4] = {
        {0,-1.756,0},
        {-1.756,0,90},
        {0,1.756,180},
        {1.756,0,270}
    };
    //automatic tag localization Approximation 
    void ATLA();

   

    double tolerance = 8.0;

    public:
    //start the automatic tag localization Approximation 
    void Start_ATLA();

    position getAtlaPosition();

    bool TagPositionDetected;

    double CameraTrust;

    // should be private
    int TagID;

    double TagPixlePosition[2];

    aprilTag curentAprilTag ;

    double CamXOffset;
    double CamYoffset;

     double pixelHieght;
     double pixleWidth;
    std::string cameraStatusOrignal = "Camera status: ";
     std::string camerastatus = cameraStatusOrignal;

      double CroboX;
    double CroboY;
};

class tracking{
    
    private:

    //how far the horosntle wheel is from the center of the robot Y axis wise
    double horizontalOffset = 3.56;

    //how far the vertial wheel is from the center of the robot x axis wise
    double verticalOffset = 2.25;

     
    
    //position RoboPosition;

    distenceData distenceList[4] = {
    //{distence, {xOfsset,Yoffset}, angleOffset}
        {0,{0,0},0},//left front (0)
        {0,{0,0},0},//right front (1)
        {0,{0,0},0},//right (2)
        {0,{0,0},0}//left (3)
    };

    void odomLoop();

    void MergeCameraAndATLA();

    position calculateDisOffset(int quad,double disOfsetX, double disOffsetY, double disDegOffset, double rA, double distence);

   

    

    public:

   

    void startOdomLoop();

    position getPositionData();

    void setPosition(double x, double y, double a);

    void getAbsolutePosition(int dis1Num, int dis2Num);
    
    void getHeaderViaDis(double perpWallHead);

    cameraTracking cameraTrack;


    // should be private

     position RoboPosition;


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
double VerticalWheelDi = 2.75;



double offsetX = 0;
double offsetY = 0;

int NumOfFails = 0;

};

extern tracking track;