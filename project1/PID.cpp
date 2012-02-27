#include "PID.h"

/** constructor with arguments for the constants structure. Defaults to having max/min value of +-1/3 for the iTerm**/
PID::PID(PIDConstants *newConstants) {
	_constants.kp = newConstants->kp;
	_constants.ki = newConstants->ki;
	_constants.kd = newConstants->kd;
	_maxValue = 1/20.0;
	_minValue = -1/20.0;
}

/** constructor with arguments for the constants structure, and min/max values for the iTerm **/
PID::PID(PIDConstants *newConstants, float max_value, float min_value) {
	_constants.kp = newConstants->kp;
	_constants.ki = newConstants->ki;
	_constants.kd = newConstants->kd;
	_maxValue = max_value;
	_minValue = min_value;
}

/** Update the PID control with a new value **/
float PID::updatePID(float error) {
	float prevError = lastError();
	//add the error to the integrator list
	addErrorToIntegrator(error);

	float pTerm, iTerm, dTerm;

	/**
	 * total term should = 1 at error = 10
	 * 
	 * pTerm should almost max out when error > 10
	 * suggested value: 0.9
	 * 
	 * iTerm should max out at maxValue/minValue and should affect the speed by that much
	 * 	i.e. if the sum is greater than 100 (the last 10 reads have been greater than 10)
	 * then the value that we get should be maxValue for iTerm
	 * suggested value: 0.5
	 * 
	 * dTerm shouldn't really do much, since the bot probably won't jump around too much and so the robot should not take too much of this into account
	 * suggested value = 0.05
	 */

	pTerm = _constants.kp * error;						//get proportional term of the PID control
	iTerm = _constants.ki * currentIntegratorError();	//get integral term of the PID control
	if (iTerm > _maxValue) {							//check that the integrator is not too high 
		iTerm = _maxValue;
    }
	else if (iTerm < _minValue) {
		iTerm = _minValue;
    }

    // multiply by the change in error (this one minus the previous)
	dTerm = _constants.kd * (error - prevError);

	//printf("pTerm = %f\n", pTerm);
	//printf("iTerm = %f\n", iTerm);
	//printf("dTerm = %f\n", dTerm);

	float gain = pTerm+iTerm+dTerm;
	if (gain > 1.0) {
		gain=1.0;
	}

	return gain;
}

/** useful when you have reached the destination, and just want to zero the error **/
void PID::flushPID() {
	for (int i = 0; i < NUM_INTEGRATOR_VALUES; i++) {
		_integratorValues[i] = 0.0;
	}
}

/** add an error to the list of the past few errors that you got
 **/
void PID::addErrorToIntegrator(float error) {
	for (int i = NUM_INTEGRATOR_VALUES-1; i > 0; i--) {
		_integratorValues[i] = _integratorValues[i-1];
	}
	_integratorValues[0] = error;
}

float PID::currentIntegratorError() {
	float error = 0.0;
	for (int i = 0; i < NUM_INTEGRATOR_VALUES; i++) {
		error += _integratorValues[i];
	}
	return error;
}

float PID::lastError() {
	return _integratorValues[0];
}

/** sets the PID's constants to new values **/
void PID::setConstants(PIDConstants *newConstants) {
	_constants.kp = newConstants->kp;
	_constants.ki = newConstants->ki;
	_constants.kd = newConstants->kd;
}
