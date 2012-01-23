#ifndef CS1567_UTILITIES_H
#define CS1567_UTILITIES_H

#define PI 3.14159265358979323846
#define DEGREE_30 0.523598776 // pi/6
#define DEGREE_150 2.617993878 // 5pi/6

#define WE_TICKS 4 // ticks per cm
#define NS_TICKS 45 // ticks per cm

#define ROBOT_DIAMETER 29 // cm

namespace Util {
	float nsToCM(float ticks);

	float weToCM(float ticks);

	float cmToNS(float cm);

	float cmToWE(float cm);
};

#endif