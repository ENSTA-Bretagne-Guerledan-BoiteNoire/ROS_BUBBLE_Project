//
// Created by tag on 28/02/17.
//

#include <cmath>

static double angle_deg(double a1,double a2){
    return fmod( a1 + a2 + 3*180, 360 ) - 180;
}

static double angle_rad(double a1,double a2){
    return (fmod( a1 + a2 + 3*M_PI, 2*M_PI ) - M_PI);
}

//static double enu2ned_yaw_rad(){
//
//}

static double ned2enu_yaw_rad(double yaw){
    return angle_rad(M_PI/2,- yaw);
}

static double ned2enu_yaw_deg(double yaw){
    return angle_deg(90,- yaw);
}

/**
 * N'est plus valable pour de sdistances supérieures à 500m
 * @param latPos
 * @param latOrigin
 * @return
 */
static double latDeg2meters(double latPos,double latOrigin){
    const double earthPerimeter = 40075000; //m
    return (latPos-latOrigin)/360*earthPerimeter;
}

/**
 * N'est plus valable pour de sdistances supérieures à 500m
 * @param longPos
 * @param latOrigin
 * @param longOrigin
 * @return
 */
static double longDeg2meters(double longPos,double latOrigin, double longOrigin){
    const double earthPerimeter = 40030000; //m
    const double Rterre = 6360000; //m
    const double circlePerimeter = Rterre*cos(latOrigin/180*M_PI)*2*M_PI;
    return (longPos-longOrigin)/360*circlePerimeter;
}

static double distance(double x1, double y1, double x2, double y2){
    return sqrt( pow(x2-x1,2) + pow(y2-y1,2) );
}