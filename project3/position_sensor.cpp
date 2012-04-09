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
#include "robot.h"
#include "constants.h"

PositionSensor::PositionSensor(Robot *robot) {
	_robot = robot;
	_pose = new Pose(0.0, 0.0, 0.0);
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
