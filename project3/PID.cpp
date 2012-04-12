/**
 * PID.cpp
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

#include "PID.h"
#include "logger.h"

PID::PID(PIDConstants *newConstants, float minError, float maxError) {
	_constants.kp = newConstants->kp;
	_constants.ki = newConstants->ki;
	_constants.kd = newConstants->kd;
	_minError = minError;
	_maxError = maxError;
	// zero out integrator error
	for (int i = 0; i < NUM_INTEGRATOR_VALUES; i++) {
		_integratorValues[i] = 0.0;
	}
}

/**************************************
 * Definition: Updates the PID control with a new value and
 *             returns the gain
 *
 * Parameters: a float error
 *
 * Returns:    a float specifying gain
 **************************************/
float PID::updatePID(float error) {
	float prevError = lastError();
	// add the error to the integrator list
	addErrorToIntegrator(error);

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
	 * dTerm shouldn't really do much, since the bot probably won't jump around too much 
	 * and so the robot should not take too much of this into account
	 * suggested value = 0.05
	 */

	// get proportional term of the PID control
	float pTerm = _constants.kp * error;						
	// get integral term of the PID control
	float iTerm = _constants.ki * currentIntegratorError();	
	// check that the integrator is not too high 
	if (iTerm > _maxError * 0.10) {							
		iTerm = 0.1;
    }
	else if (iTerm < _minError * 0.10) {
		iTerm = -0.1;
    }

    // get differential term by multiplying by the change in error 
	float dTerm = _constants.kd * (error - prevError);

    LOG.write(LOG_LOW, "pid terms", "pTerm = %f", pTerm);
	LOG.write(LOG_LOW, "pid terms", "iTerm = %f", iTerm);
	LOG.write(LOG_LOW, "pid terms", "dTerm = %f", dTerm);

	float gain = pTerm + iTerm + dTerm;
	if (gain > 1.0) {
		gain = 1.0;
	}
	else if (gain < 0.0) {
		gain = 0.0;
	}

	return gain;
}

/**************************************
 * Definition: Clears integrator by zeroing it out. Useful when you have
 *             reached the destination and want to remove the error
 **************************************/
void PID::flushPID() {
	for (int i = 0; i < NUM_INTEGRATOR_VALUES; i++) {
		_integratorValues[i] = 0.0;
	}
}

/**************************************
 * Definition: Adds an error to the fixed-size integrator list 
 *
 * Parameters: a float error
 **************************************/
void PID::addErrorToIntegrator(float error) {
	for (int i = NUM_INTEGRATOR_VALUES-1; i > 0; i--) {
		_integratorValues[i] = _integratorValues[i-1];
	}
	_integratorValues[0] = error;
}

/**************************************
 * Definition: Returns the current integrator error (sum of errors)
 *
 * Returns:    a float error representing the sum of integrator errors
 **************************************/
float PID::currentIntegratorError() {
	float error = 0.0;
	for (int i = 0; i < NUM_INTEGRATOR_VALUES; i++) {
		error += _integratorValues[i];
	}
	return error;
}

/**************************************
 * Definition: Returns the most recent error
 *
 * Returns:    a float error
 **************************************/
float PID::lastError() {
	return _integratorValues[0];
}

/**************************************
 * Definition: Updates the PID's constants
 *
 * Parameters: a pointer to a PIDConstants struct
 **************************************/
void PID::setConstants(PIDConstants *newConstants) {
	_constants.kp = newConstants->kp;
	_constants.ki = newConstants->ki;
	_constants.kd = newConstants->kd;
}
