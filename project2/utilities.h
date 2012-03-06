#ifndef CS1567_UTILITIES_H
#define CS1567_UTILITIES_H

#include <string>

#define NUM_ROBOTS 6

#define ROSIE 0
#define BENDER 1
#define JOHNNY5 2
#define OPTIMUS 3
#define WALLE 4
#define GORT 5

const std::string ROBOTS[] = {
    "rosie",
    "bender",
    "johnny5",
    "optimus",
    "walle",
    "gort"
};

const std::string ROBOT_ADDRESSES[] = {
    "192.168.1.41",
    "192.168.1.42",
    "192.168.1.43",
    "192.168.1.44",
    "192.168.1.45",
    "192.168.1.46"
};

namespace Util {
    void matrixMult(float *mA, int lA, int hA, 
                    float *mB, int lB, int hB, 
                    float *mC);

    float normalizeThetaError(float thetaError);
    float normalizeTheta(float theta);

    int nameFrom(std::string);
};

#endif
