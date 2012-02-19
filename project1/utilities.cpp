#include "utilities.h"
#include "constants.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

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

	void mMult(float *mA, int lA, int hA, float *mB, int lB, int hB, float *mC) {
	    if (hA != lB) {
	    	printf("Inner matrix dimensions do not match!\n");
			return;
	    }
		
            //float mC[lA][hB];
			
	    for (int i = 0; i < lA; i++) {
	    	for (int j = 0; j < hB; j++) {
		    	mC[j + (i*hB)] = 0;
		    	for(int k = 0; k < lB; k++) {
	    	    	mC[j+(i*hB)] += mA[(i*hB)+k] * mB[(k*hB)+j];
			//printf("mC[%d+(%d*%d)=%f] += (mA[(%d*%d)+%d] = %f * mB[(%d*%d)+%d] = %f) = %f\n",j,i,hB,mC[j+(i*hB)],i,hB,k,mA[(i*hB)+k],k,hB,j,mB[(k*hB)+j]);
			//printf("i: %d, j:%d, k:%d\n",i,j,k);
		    	}
			}
	    }
	    //return *mC;
	}

	float normalizeThetaError(float thetaError) {
		while (thetaError <= -PI) {
            thetaError += 2*PI;
        }
        while (thetaError >= PI) {
            thetaError -= 2*PI;
        }
        return thetaError;
	}

	/* Input: A number in range [-inf, inf] (usually [-pi, pi])
	   Returns: A number in range [0, 2pi]
	*/
	float normalizeTheta(float theta) {
		theta = fmod(theta, 2*PI);
        while (theta < 0) {
            theta += 2*PI;
        }
        return theta;
	}
};
