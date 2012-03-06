#include "position_sensor.h"
#include "constants.h"

PositionSensor::PositionSensor(RobotInterface *robotInterface) {
	_robotInterface = robotInterface;
	_pose = new Pose(0, 0, 0);
	_passed2PI = false;
}

PositionSensor::~PositionSensor() {
	delete _pose;
}

void PositionSensor::resetPose(Pose *pose) {
	_pose->setX(pose->getX());
	_pose->setY(pose->getY());
	_pose->setTheta(pose->getTheta());
	_pose->setNumRotations(pose->getNumRotations());
}

float PositionSensor::getX() {
	return getPose()->getX();
}

float PositionSensor::getY() {
	return getPose()->getY();
}

float PositionSensor::getTheta() {
	return getPose()->getTheta();
}

Pose* PositionSensor::getPose() {
	return _pose;
}

/* This method should be called by inherited class
   after each update to keep
   total theta in check by tracking how many rotations
   passed 2PI the sensor has read 
*/
void PositionSensor::_adjustTotalTheta(float theta) {
	float lastTheta = getTheta();

    if ((lastTheta > (3/2.0)*PI && theta < PI/2.0) || 
        (lastTheta < PI/2.0 && theta > (3/2.0)*PI)) {
        _passed2PI = !_passed2PI;
    }
    
    if (_passed2PI) {
    	if (lastTheta > PI && theta < PI) {
        	_pose->modifyRotations(1);
        	_passed2PI = false;
    	}
    	else if (lastTheta < PI && theta > PI) {
    		_pose->modifyRotations(-1);
    		_passed2PI = false;
    	}
    }
}