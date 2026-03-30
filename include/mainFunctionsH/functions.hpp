#include "math.h"
//#include "main.h"
//#include "mainFunctionsH\tracking.hpp"
class functions{
    private:
    public:

    double DegToRad( double D);

    double RadToDeg (double R);

    //normalize Radian angles
    double normalizeAngle(double R);

    //normalize degree angles
    double normalizeDegAngle(double A);

    //round to the nearist thousandth
    double roundNearistThous(double Num);

    //swithces mm to inches
    double MM_to_IN(double MM);

};

//so any files that includes this one can use the classes
extern functions function;
