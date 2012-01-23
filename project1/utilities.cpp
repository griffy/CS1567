#include "utilities.h"

namespace Util {
	float nsToCM(float ticks) {
	    return ticks / NS_TICKS;
	}

	float weToCM(float ticks) {
	    return ticks / WE_TICKS;
	}

	float cmToNS(float cm) {
	    return cm * NS_TICKS;
	}

	float cmToWE(float cm) {
	    return cm * WE_TICKS;
	}
};