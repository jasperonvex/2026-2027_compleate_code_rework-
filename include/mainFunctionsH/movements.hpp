#include"vector"
struct Waypoint
{
    double x;
    double y;
    float maxVeo;
    float lookAhed;
};


struct pathD
{
    double x;
    double y;
    double curvature;
    double disAlongPoint;
    double targetVel;
    double lookAhead;
};

class Movement{
    private:

    // settings for purepursuit
    double trackWidth = 0;
    double Kv = 0;
    double Ka = 0;
    double Kp = 0;
    //

    double disPerPoint = 0.5;

    std::vector<pathD> path;
    public:

    
    void PurePuresuit(bool Isreverse, double timeout, std::vector<Waypoint> Waypoints);

    void turn(double Deg);

    void turnToPos(double x, double y);

    void turnFacing(double Head);

};

//std::vector<Waypoint> waypoints;



