#include "PID.h"
/**
 * @author Shawn Hanna
 * @group_number 1
 * @date 2/7/2012
**/

/** constructor with arguments for the constants structure. Defaults to having max/min value of +-1/3 for the iTerm**/
PID::PID(PIDConstants* newConstants){
	constants.kp=newConstants->kp;
	constants.ki=newConstants->ki;
	constants.kd=newConstants->kd;
	maxValue=1/3.0;
	minValue=-1/3.0;
}

/** constructor with arguments for the constants structure, and min/max values for the iTerm **/
PID::PID(PIDConstants* newConstants, float max_value, float min_value){
	constants.kp=newConstants->kp;
	constants.ki=newConstants->ki;
	constants.kd=newConstants->kd;
	maxValue=max_value;
	minValue=min_value;
}

/** Update the PID control with a new value **/
float PID::updatePID(float error){
	//add the error to the integrator list
	addErrorToIntegrator(error);

	float pTerm, iTerm, dTerm;

	pTerm=constants.kp*error;					//get proportional term of the PID control
	iTerm=constants.ki*currentIntegratorError;	//get integral term of the PID control
	if(iTerm>maxValue)							//check that the integrator is not too high 
		iTerm=maxValue;
	else if(iTerm<minValue)
		iTerm=minValue;
	dTerm=constants.kd*(error-integratorValues[1]);

	printf("pTerm = %f\n",pTerm);
	printf("iTerm = %f\n",iTerm);
	printf("dTerm = %f\n",dTerm);

	return (pTerm+iTerm+dTerm);
}

/** useful when you have reached the destination, and just want to zero the error **/
void PID::flushPID() {
	for(int i=0; i<NUM_INTEGRATOR_VALUES; i++){
		integratorValues[i]=0.0;
	}
}

/** add an error to the list of the past few errors that you got
 ** @return the current integrated error
 **/
float PID::addErrorToIntegrator(float error){
	currentIntegratorError=error;
	for(int i=1; i<NUM_INTEGRATOR_VALUES; i++){
		integratorValues[i]=integratorValues[i-1];
		currentIntegratorError+=integratorValues[i];
	}
	return currentIntegratorError;
}

/** sets the PID's constants to new values **/
void PID::setConstants(PIDConstants* newConstants){
	constants.kp=newConstants->kp;
	constants.ki=newConstants->ki;
	constants.kd=newConstants->kd;
}
