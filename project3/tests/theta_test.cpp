#include <cmath>
#include <cstdio>
#include "utilities.h"

int main() {
    // dist is 202.237
    float goalX = 300.0;
    float goalY = 0.0;
    float curX = 100.0;
    float curY = 30.0;
    printf("goal: (%f, %f)\n", goalX, goalY);
    printf("cur: (%f, %f)\n", curX, curY);

    float yError = goalY - curY;
    float xError = goalX - curX;
    float thetaDesired = atan2(yError, xError);
    printf("atan2: %f\n", thetaDesired);

    thetaDesired = Util::normalizeTheta(thetaDesired);
    printf("normalized: %f\n", thetaDesired);

    float thetaError = thetaDesired - 0.0;
    thetaError = Util::normalizeThetaError(thetaError);
    printf("error1: %f\n", thetaError);

    thetaError = thetaDesired - 6.28318531;
    thetaError = Util::normalizeThetaError(thetaError);
    printf("error2: %f\n", thetaError);

    thetaDesired = -0.795;
    thetaDesired = Util::normalizeTheta(thetaDesired);
    printf("goal: %f\n", thetaDesired);

    return 0;
}
