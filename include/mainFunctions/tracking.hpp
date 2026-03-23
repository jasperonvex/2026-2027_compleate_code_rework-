 struct position
    {
       double x;
       double y;
       double a;
    };

class tracking{
    private:
   

    position RoboPosition;

    void odomLoop();

    
    public:

    void startOdomLoop();

    position getPositionData();

    void getDistencePosition();
    

};