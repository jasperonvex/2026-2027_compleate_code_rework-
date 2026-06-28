 
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
   }
    cameraoffset = {0,0,0,0};

    double Image_Width = 320;
    double Image_Height = 240;

    double Horosontle_FOV = 74;
    double Veritcal_FOV = 63;

    

    double Focal_length_Pixels = 157.3605;// (distence * pixlewidth) / actual width
    
    aprilTag apriltags[5] = {
        {0,{0,0}},
        {1,{-48,24}},
        {2,{-48,-24}},
        {3,{-24,-48}},
        {4,{24,-48}}
    };

    //distance from tag to center of the goal is 1.756
    position aprilTag_offset[4] = {
        {0,1.756,0},
        {1.756,0,90},
        {0,-1.756,180},
        {-1.756,0,270}
    };
    //automatic tag localization Approximation 
    void ATLA();

    public:
    //start the automatic tag localization Approximation 
    void Start_ATLA();

    int TagID;

    double TagPixlePosition[2];
};

class tracking{
    private:

    //how far the horosntle wheel is from the center of the robot Y axis wise
    double horizontalOffset = 0;

    //how far the vertial wheel is from the center of the robot x axis wise
    double verticalOffset = 0;

     
    
    //position RoboPosition;

    distenceData distenceList[4] = {
    //{distence, {xOfsset,Yoffset}, angleOffset}
        {0,{0,0},0},//left front (0)
        {0,{0,0},0},//right front (1)
        {0,{0,0},0},//right (2)
        {0,{0,0},0}//left (3)
    };

    void odomLoop();

    position calculateDisOffset(int quad,double disOfsetX, double disOffsetY, double disDegOffset, double rA, double distence);

    
    public:

    position RoboPosition;

    void startOdomLoop();

    position getPositionData();

    void setPosition(double x, double y, double a);

    void getAbsolutePosition(int dis1Num, int dis2Num);
    
    void getHeaderViaDis(double perpWallHead);

    cameraTracking cameraTrack;

};

extern tracking track;