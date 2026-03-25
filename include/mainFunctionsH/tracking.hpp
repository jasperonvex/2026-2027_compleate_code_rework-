 
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

class tracking{
    private:

    //how far the horosntle wheel is from the center of the robot Y axis wise
    double horizontalOffset = 0;

    //how far the vertial wheel is from the center of the robot x axis wise
    double verticalOffset = 0;

     
    
    position RoboPosition;

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

    void startOdomLoop();

    position getPositionData();

    void setPosition(double x, double y, double a);

    void getPosishViaDis(int dis1Num, int dis2Num);
    
    void getHeaderViaDis(double perpWallHead);

};

