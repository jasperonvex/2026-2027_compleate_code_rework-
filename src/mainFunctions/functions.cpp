#include "mainFunctions\functions.hpp"


functions function;

double functions::normalizeAngle(double R){
    double angle = R;
    while(angle > M_PI) { angle -= 2 * M_PI;}
    while(angle < -M_PI){ angle += 2 * M_PI;}
    return  angle;
}

double functions::normalizeDegAngle(double A){
    double angle = A;
    while(angle > 180) { angle -= 360;}
    while(angle <= -180){ angle += 360;}
    return  angle;
}


