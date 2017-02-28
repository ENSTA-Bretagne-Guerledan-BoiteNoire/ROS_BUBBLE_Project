//
// Created by tag on 28/02/17.
//

static double angle(double a1,double a2){

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
    return (longPos-longOrigin)*Rterre*cosd(latOrigin)/360*2*M_PI;
}