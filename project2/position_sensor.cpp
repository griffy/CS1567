/**
 * position_sensor.cpp
 * 
 * @brief 
 *      This is an abstract class intended to be inherited from
 *      by a sensor class that keeps a global pose.
 * 
 * @author
 *      Shawn Hanna
 *      Tom Nason
 *      Joel Griffith
 *
 **/

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

/**************************************
 * Definition: Sets the stored pose to the given one
 *
 * Parameters: a pose object
 **************************************/
void PositionSensor::resetPose(Pose *pose) {
	_pose->setX(pose->getX());
	_pose->setY(pose->getY());
	_pose->setTheta(pose->getTheta());
	_pose->setNumRotations(pose->getNumRotations());
}

/**************************************
 * Definition: Returns the current x
 *
 * Returns:    x as a float
 **************************************/
float PositionSensor::getX() {
	return getPose()->getX();
}

/**************************************
 * Definition: Returns the current y
 *
 * Returns:    y as a float
 **************************************/
float PositionSensor::getY() {
	return getPose()->getY();
}

/**************************************
 * Definition: Returns the current theta
 *
 * Returns:    theta as a float
 **************************************/
float PositionSensor::getTheta() {
	return getPose()->getTheta();
}

/**************************************
 * Definition: Returns the current pose
 *
 * Returns:    a pointer to the stored pose
 **************************************/
Pose* PositionSensor::getPose() {
	return _pose;
}

/**************************************
 * Definition: Adjusts total theta by checking for
 *             rotations (past 2PI) since the last update.
 *
 * Note:       this method should be called by inherited class
 *             after each update to keep thetas in check
 **************************************/
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
