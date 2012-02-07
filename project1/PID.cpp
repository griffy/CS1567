#include "PID.h"

PID::PID(PIDConstants* newConstants){
	constants.kp=newConstants->kp;
	constants.ki=newConstants->ki;
	constants.kd=newConstants->kd;
	maxValue=1/3.0;
	minValue=-1/3.0;
}

PID::PID(PIDConstants* newConstants, float max_value, float min_value){
	constants.kp=newConstants->kp;
	constants.ki=newConstants->ki;
	constants.kd=newConstants->kd;
	maxValue=max_value;
	minValue=min_value;
}

float PID::updatePID(float error){
	//find the error from x/y

	addErrorToIntegrator(error);

	float pTerm, iTerm, dTerm;

	pTerm=constants.kp*error;
	iTerm=constants.ki*currentIntegratorError;
	if(iTerm>maxValue)
		iTerm=maxValue;
	else if(iTerm<minValue)
		iTerm=minValue;
	dTerm=constants.kd*(error-integratorValues[1]);

	printf("pTerm = %f\n",pTerm);
	printf("iTerm = %f\n",iTerm);
	printf("dTerm = %f\n",dTerm);

	return (pTerm+iTerm+dTerm);
}

float PID::addErrorToIntegrator(float error){
	currentIntegratorError=error;
	for(int i=1; i<NUM_INTEGRATOR_VALUES; i++){
		integratorValues[i]=integratorValues[i-1];
		currentIntegratorError+=integratorValues[i];
	}
	return currentIntegratorError;
}

void PID::setConstants(PIDConstants* newConstants){
	constants.kp=newConstants->kp;
	constants.ki=newConstants->ki;
	constants.kd=newConstants->kd;
}
