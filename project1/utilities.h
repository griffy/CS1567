#ifndef CS1567_UTILITIES_H
#define CS1567_UTILITIES_H

namespace Util {
	float nsToCM(float ticks); // converts North Star ticks to cm

	float weToCM(float ticks); // converts Wheel Encoder ticks to cm

	float cmToNS(float cm); // converts cm to North Star ticks

	float cmToWE(float cm); // converts cm to Wheel Encoder ticks

	void mMult(float (*mA), int lA, int hA, float (*mB), int lB, int hB, float (*mC));
	
	void printOpeningDialog(std::string name);
};

#endif
