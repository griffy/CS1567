/**
 * wheel_encoders.cpp
 * 
 * @brief 
 * 		This class performs all functions for the wheel encoder sensors (except for when to update the data)
 * 		It stores the raw and filtered data, and contains functions for converting the data into global coordinates
 * 
 * @author
 * 		Joel Griffith
 * 		Shawn Hanna
 * 		Tom Nason
 * 
 * @date
 * 		created - 2/2/2012
 * 		modified - 3/24/2012
 **/

#include "wheel_encoders.h"
#include "constants.h"
#include "utilities.h"
#include "logger.h"

WheelEncoders::WheelEncoders(RobotInterface *robotInterface)
: PositionSensor(robotInterface) {
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
* Definition: - Translates incremental wheel encoder data to global coordinate system and updates robot pose
*
* Parameters: room - Integer corresponding to robot's NorthStar room
*
* Notes: Requires the interface be updated prior to calling 
************************************************/
void WheelEncoders::updatePose(int room) {
	LOG.printfScreen(LOG_LOW, "WE pose update", "x: %f deltaX: %f y: %f deltaY: %f\n", getX(), _getDeltaX(), getY(), _getDeltaY());
	float x = getX() + _getDeltaX();
	float y = getY() + _getDeltaY();
	float theta = Util::normalizeTheta(getTheta() + _getDeltaTheta());

	_adjustTotalTheta(theta);

	LOG.printfFile(LOG_LOW, "WE_positions_clean", "%f, %f, %f, %f, %f, %f\n", getX(), _getDeltaX(), getY(), _getDeltaY(), getTheta(), _getDeltaTheta());
	
	LOG.printfFile(LOG_LOW, "WE_positions_raw", "%f, %f, %f\n", _getFilteredDeltaLeft(), _getFilteredDeltaRight(), _getFilteredDeltaRear());
	_pose->setX(x);
	_pose->setY(y);
	_pose->setTheta(theta);
}

/************************************************
 * Definition: - Translates wheel encoder motion from robot axis into the corresponding change in X in the global coordinate system  
 *
 * Returns: rotatedDeltaX -  delta x in terms of global coordinate system 
 ***********************************************/
float WheelEncoders::_getDeltaX() {
	float scaledDeltaX = _getRobotDeltaY() / WE_SCALE;
	float rotatedDeltaX = scaledDeltaX * cos(getTheta());
    return rotatedDeltaX;
}

/************************************************
 * Definition: - Translates wheel encoder motion from robot axis into the corresponding change in Y in the global coordinate system  
 *
 * Returns: rotatedDeltaY -  delta y in terms of global coordinate system 
 ***********************************************/
float WheelEncoders::_getDeltaY() {
	float scaledDeltaY = _getRobotDeltaY() / WE_SCALE;
	float rotatedDeltaY = scaledDeltaY * sin(getTheta());
    return rotatedDeltaY;
}

/************************************************
 * Definition: - Translates wheel encoder motion from robot axis into the corresponding change in theta in the global coordinate system  
 *
 * Returns: rotatedDeltaTheta -  delta theta in terms of global coordinate system 
 ***********************************************/
float WheelEncoders::_getDeltaTheta() {
	float rearRobotDeltaX = _getFilteredDeltaRear();
	float scaledRobotDeltaX = rearRobotDeltaX / WE_SCALE;
	return -scaledRobotDeltaX / (ROBOT_DIAMETER / 2.0);
}

/************************************************
 * Definition: - Translates wheel encoder ticks into the robot coordinate system  
 *
 * Returns: avgRobotDeltaY - Change in Y position in robot coordinate system
 ***********************************************/
float WheelEncoders::_getRobotDeltaY() {
    float leftRobotDeltaY = -_getFilteredDeltaLeft() * cos(DEGREE_150);
    float rightRobotDeltaY = _getFilteredDeltaRight() * cos(DEGREE_30);
    float avgRobotDeltaY = (leftRobotDeltaY + rightRobotDeltaY) / 2.0;
    return avgRobotDeltaY;
}

/************************************************
 * Definition: - Translates wheel encoder ticks into the robot coordinate system  
 *
 * Returns: Filtered wheel encoder (delta) ticks for the left wheel
 ***********************************************/
float WheelEncoders::_getFilteredDeltaLeft() {
    int left = _robotInterface->getWheelEncoder(RI_WHEEL_LEFT);
    return _filterLeft->filter((float) left);
}

// Returns: filtered wheel encoder (delta) ticks for the right wheel
/************************************************
 * Definition: - Translates wheel encoder ticks into the robot coordinate system  
 *
 * Returns: avgRobotDeltaY - Change in Y position in robot coordinate system
 ***********************************************/
float WheelEncoders::_getFilteredDeltaRight() {
    int right = _robotInterface->getWheelEncoder(RI_WHEEL_RIGHT);
    return _filterRight->filter((float) right);
}

// Returns: filtered wheel encoder (delta) ticks for the rear wheel
/************************************************
 * Definition: - Translates wheel encoder ticks into the robot coordinate system  
 *
 * Returns: avgRobotDeltaY - Change in Y position in robot coordinate system
 ***********************************************/
float WheelEncoders::_getFilteredDeltaRear() {
    int rear = _robotInterface->getWheelEncoder(RI_WHEEL_REAR);
    return _filterRear->filter((float) rear);
}
