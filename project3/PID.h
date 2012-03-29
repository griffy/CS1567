/**
 * PID.h
 * 
 * @brief 
 * 		This class is a basic PID controller.  It creates, stores, 
 *      and returns the value from the PID.
 * 
 * @author
 *      Shawn Hanna
 *      Tom Nason
 *      Joel Griffith
 * 
 **/

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
	PID(PIDConstants *constants);
	PID(PIDConstants *constants, float minValue, float maxValue);
	float updatePID(float error);
	void flushPID();
	void addErrorToIntegrator(float error);
	float currentIntegratorError();
	float lastError();
	void setConstants(PIDConstants *newConstants);
private:
	float _integratorValues[NUM_INTEGRATOR_VALUES];
	PIDConstants _constants;
	float _minValue;
	float _maxValue;
};

#endif
