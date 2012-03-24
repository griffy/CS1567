#ifndef CS1567_PID_H
#define CS1567_PID_H

#include "pose.h"
#include <stdio.h>
#include <stdlib.h>

#define NUM_INTEGRATOR_VALUES 10

typedef struct {
	float ki, kp, kd;
} PIDConstants;

class PID {
public:
	PID(PIDConstants *constants, float max_value, float min_value);
	PID(PIDConstants *constants);
	
	void flushPID();
	
	void setConstants(PIDConstants *newConstants);
	
	// gets the speed you want to travel at based on your current Pose and the destination Pose
	float updatePID(float error);

	// adds the error to the list of previous values.
	void addErrorToIntegrator(float error);

	float currentIntegratorError(); // most recent sum of the integratorValues

	float lastError(); // returns the error at index 0
private:
	float _integratorValues[NUM_INTEGRATOR_VALUES]; //most recent errors (index 0 is the most recent)
	PIDConstants _constants;
	float _maxValue;
	float _minValue;
};

#endif
