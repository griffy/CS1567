#ifndef CS1567_UTILITIES_H
#define CS1567_UTILITIES_H

#include <string>

#define DEFAULT_IMP 0

#define HIGH_IMP 2
#define MEDIUM_IMP 1
#define LOW_IMP 0

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
    float nsToCM(float ticks); // converts North Star ticks to cm

    float weToCM(float ticks); // converts Wheel Encoder ticks to cm

    float cmToNS(float cm); // converts cm to North Star ticks

    float cmToWE(float cm); // converts cm to Wheel Encoder ticks

    void mMult(float (*mA), int lA, int hA, float (*mB), int lB, int hB, float (*mC));

    float normalizeThetaError(float thetaError);
    float normalizeTheta(float theta);

    int nameFrom(std::string);
};

#endif
