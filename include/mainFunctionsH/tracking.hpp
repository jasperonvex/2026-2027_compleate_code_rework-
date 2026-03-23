 
 struct position
    {
       double x;
       double y;
       double a;
    };

class tracking{
    private:

    //how far the horosntle wheel is from the center of the robot Y axis wise
    double horizontalOffset = 0;

    //how far the vertial wheel is from the center of the robot x axis wise
    double verticalOffset = 0;

    position RoboPosition;

    void odomLoop();

    
    public:

    void startOdomLoop();

    position getPositionData();

    void setPosition(double x, double y, double a);

    void getDistencePosition();
    
    void getHeaderViaDis();

};