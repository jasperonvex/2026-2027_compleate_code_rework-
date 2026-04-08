#include "mainFunctionsH\functions.hpp"


functions function;

double functions::DegToRad(double D){
    return D * (M_PI / 180);
}

double functions::RadToDeg(double R){
    return R * (180 / M_PI);
}

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

double functions::roundNearistThous(double Num){
    double n = Num;
    n = round(n*1000);
    n = n/1000;
    return n;
}

double functions::MM_to_IN(double MM){
    return MM/25.4;
}

double functions::GetDistence(double x1, double y1, double x2, double y2){
    return sqrt(pow(x1-x2,2) + pow(y1-y2,2));
}
