#include "math.h"
class functions{
    private:
    public:

    //normalize Radian angles
    double normalizeAngle(double R);
    //normalize degree angles
    double normalizeDegAngle(double A);
};

//so any files that includes this one can use the classes
extern functions function;