/**
 * wheel_encoders.cpp
 * 
 * @brief 
 * 		This class inherits from the position sensor class.
 *      It performs all functions for the wheel encoders
 *      (except for when to update the data). It keeps a pose, stores 
 *      the raw and filtered wheel encoder data, and contains functions for converting 
 *      the data into global coordinates to be stored back in the pose.
 * 
 * @author
 * 		Shawn Hanna
 * 		Tom Nason
 * 		Joel Griffith
 *
 **/

#include "wheel_encoders.h"
#include "constants.h"
#include "utilities.h"
#include "logger.h"
#include <robot_if++.h>
#include "robot.h"

WheelEncoders::WheelEncoders(Robot *robot)
: PositionSensor(robot) {
	_filterLeft = new FIRFilter("filters/we.ffc");
	_filterRight = new FIRFilter("filters/we.ffc");
	_filterRear = new FIRFilter("filters/we.ffc");
}

WheelEncoders::~WheelEncoders() {
	delete _filterLeft;
	delete _filterRight;
	delete _filterRear;
}

/************************************************
* Definition: Translates incremental wheel encoder data to 
*             global coordinate system and updates robot pose
*
* Note:       Requires the interface be updated prior to calling 
************************************************/
void WheelEncoders::updatePose() {
	LOG.write(LOG_LOW, "WE_positions_raw", 
		      "we update (raw): left: %f right: %f rear: %f", 
		      _getFilteredDeltaLeft(), _getFilteredDeltaRight(), _getFilteredDeltaRear());
	LOG.write(LOG_LOW, "wheelEncodersUpdate", 
		      "we update: x: %f deltaX: %f y: %f deltaY: %f", 
		      getX(), _getDeltaX(), getY(), _getDeltaY());

	float x = getX() + _getDeltaX();
	float y = getY() + _getDeltaY();
	float theta = Util::normalizeTheta(getTheta() + _getDeltaTheta());

	_pose->setX(x);
	_pose->setY(y);
	_pose->setTheta(theta);
}

/************************************************
 * Definition: Translates wheel encoder motion from robot axis 
 *             into the corresponding change in X in the global coordinate system  
 *
 * Returns:    delta x in terms of global coordinate system 
 ***********************************************/
float WheelEncoders::_getDeltaX() {
	float scaledDeltaX = _getRobotDeltaY() / WE_SCALE;
	float rotatedDeltaX = scaledDeltaX * cos(getTheta());
    return rotatedDeltaX;
}

/************************************************
 * Definition: Translates wheel encoder motion from robot axis 
 *             into the corresponding change in Y in the global coordinate system  
 *
 * Returns:    delta y in terms of global coordinate system 
 ***********************************************/
float WheelEncoders::_getDeltaY() {
	float scaledDeltaY = _getRobotDeltaY() / WE_SCALE;
	float rotatedDeltaY = scaledDeltaY * sin(getTheta());
    return rotatedDeltaY;
}

/************************************************
 * Definition: Translates wheel encoder motion from robot axis 
 *             into the corresponding change in theta in the global coordinate system  
 *
 * Returns:    delta theta in terms of global coordinate system 
 ***********************************************/
float WheelEncoders::_getDeltaTheta() {
	float rearRobotDeltaX = _getFilteredDeltaRear();
	float scaledRobotDeltaX = rearRobotDeltaX / WE_SCALE;
	return -scaledRobotDeltaX / (ROBOT_DIAMETER / 2.0);
}

/************************************************
 * Definition: Translates wheel encoder ticks into the robot coordinate system  
 *
 * Returns:    delta y in robot coordinate system
 ***********************************************/
float WheelEncoders::_getRobotDeltaY() {
    float leftRobotDeltaY = -_getFilteredDeltaLeft() * cos(DEGREE_150);
    float rightRobotDeltaY = _getFilteredDeltaRight() * cos(DEGREE_30);
    // some robots have bad wheel encoders for one side,
    // so account for this by faking the data on the opposite
    // wheel
    switch (_robot->getName()) {
    case ROSIE:
    	rightRobotDeltaY = leftRobotDeltaY;
    	break;
    case OPTIMUS:
    	leftRobotDeltaY = rightRobotDeltaY;
    	break;
    }
    float avgRobotDeltaY = (leftRobotDeltaY + rightRobotDeltaY) / 2.0;
    return avgRobotDeltaY;
}

/************************************************
 * Definition: Returns filtered wheel encoder ticks for left wheel
 *
 * Returns:    delta left ticks
 ***********************************************/
float WheelEncoders::_getFilteredDeltaLeft() {
    int left = _robot->getInterface()->getWheelEncoder(RI_WHEEL_LEFT);
    return _filterLeft->filter((float) left);
}

/************************************************
 * Definition: Returns filtered wheel encoder ticks for right wheel
 *
 * Returns:    delta right ticks
 ***********************************************/
float WheelEncoders::_getFilteredDeltaRight() {
    int right = _robot->getInterface()->getWheelEncoder(RI_WHEEL_RIGHT);
    return _filterRight->filter((float) right);
}

/************************************************
 * Definition: Returns filtered wheel encoder ticks for rear wheel
 *
 * Returns:    delta rear ticks
 ***********************************************/
float WheelEncoders::_getFilteredDeltaRear() {
    int rear = _robot->getInterface()->getWheelEncoder(RI_WHEEL_REAR);
    return _filterRear->filter((float) rear);
}
