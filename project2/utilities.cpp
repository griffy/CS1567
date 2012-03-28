/**
 * utilities.cpp
 * 
 * @brief 
 * 		This namespace contains commonly used functions that can be used throughout the program
 * 
 * @author
 *      Shawn Hanna
 *      Tom Nason
 *      Joel Griffith
 * 
 **/

#include "utilities.h"
#include "constants.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <algorithm>

namespace Util {
    /**************************************
     * Definition: Performs a matrix multiplication of two matrices, A and B,
     *             and stores the result in a third matrix, C
     *
     * Parameters: float pointer matrix A, length and height of A as ints
     *             float pointer matrix B, length and height of B as ints
     *             float pointer matrix C
     **************************************/
    void matrixMult(float *mA, int lA, int hA, float *mB, int lB, int hB, float *mC) {
        if (hA != lB) {
            printf("Inner matrix dimensions do not match!\n");
            return;
        }

        for (int i = 0; i < lA; i++) {
            for (int j = 0; j < hB; j++) {
                mC[j + (i*hB)] = 0;
                for (int k = 0; k < lB; k++) {
                    mC[j+(i*hB)] += mA[(i*hB)+k] * mB[(k*hB)+j];
                }
            }
        }
    }

    /**************************************
     * Definition: Takes a theta error, possibly negative, and
     *             converts it to the appropriate representation [0, 2PI]
     *
     * Parameters: float specifying theta error
     *
     * Returns:    new float theta error
     **************************************/
    float normalizeThetaError(float thetaError) {
        while (thetaError <= -PI) {
            thetaError += 2*PI;
        }
        while (thetaError >= PI) {
            thetaError -= 2*PI;
        }
        return thetaError;
    }

    /**************************************
     * Definition: Takes a theta (usually [-pi, pi]) and 
     *             normalizes it into range [0, 2PI]
     *
     * Parameters: float specifying theta
     *
     * Returns:    new float theta
     **************************************/
    float normalizeTheta(float theta) {
        while (theta >= 2*PI) {
            theta -= 2*PI;
        }
        while (theta < 0) {
            theta += 2*PI;
        }
        return theta;
    }

    /**************************************
     * Definition: Returns an integer referring to the name
     *             of a robot based on its address
     *
     * Parameters: string with robot's address (ip or hostname)
     *
     * Returns:    int specifying robot's name
     **************************************/
    int nameFrom(std::string address) {
        for (int i = 0; i < NUM_ROBOTS; i++) {
            if (ROBOTS[i] == address) {
                return i;
            }
        }
        for (int i = 0; i < NUM_ROBOTS; i++) {
            if (ROBOT_ADDRESSES[i] == address) {
                return i;
            }
        }
    }
    
    /**************************************
     * Definition: Maps a value from the first range to the second
     *
     * Parameters: float specifying the value to map, and the
     *             left (range) min and max as floats, along with the 
     *             right (range) min and max as floats.
     *
     * Returns:    mapped value as a float
     **************************************/
    float mapValue(float value, float leftMin, float leftMax, float rightMin, float rightMax) {
		float leftSpan = leftMax - leftMin;
		float rightSpan = rightMax - rightMin;
		float valueScaled = (value - leftMin) / (leftSpan);
		return rightMin + (valueScaled * rightSpan);
	}

    /**************************************
     * Definition: Takes a robot speed and makes sure it falls in
     *             range [0, cap]
     *
     * Parameters: ints specifying the speed and the cap
     *
     * Returns:    capped speed as an int
     **************************************/
    int capSpeed(int speed, int cap) {
        return std::min(std::max(speed, 0), cap);
    }
};
