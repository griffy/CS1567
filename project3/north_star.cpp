/**
 * north_star.cpp
 * 
 * @brief 
 * 		This class inherits from the position sensor class.
 *      It performs all functions for the north star sensors 
 *      (except for when to update the data). It keeps a pose, stores 
 *      the raw and filtered north star data, and contains functions for converting 
 *      the data into global coordinates to be stored back in the pose.
 * 
 * @author
 * 		Shawn Hanna
 * 		Tom Nason
 * 		Joel Griffith
 *
 **/

#include "north_star.h"
#include "constants.h"
#include "logger.h"
#include "utilities.h"
#include "robot.h"
 
NorthStar::NorthStar(Robot *robot)
: PositionSensor(robot), _oldX(), _oldY() {
	_lastRoom = -1;

	_filterX = new FIRFilter("filters/ns_x.ffc");
	_filterY = new FIRFilter("filters/ns_y.ffc");
	_filterTheta = new FIRFilter("filters/ns_theta.ffc");
	
	_oldX.resize(_filterX->getOrder(), 0);
	_oldY.resize(_filterY->getOrder(), 0);
}

NorthStar::~NorthStar() {
	delete _filterX;
	delete _filterY;
	delete _filterTheta;
}

/**************************************************
 * Definition: This method serves to pull new NorthStar values from the 
 *             robot and transform them into the global coordinate system.
 *
 * 		       Specific corrections for room changes and non-linearity are 
 *             included here as well
 *
 * Note:       Requires the interface be updated prior to calling 
 *************************************************/
void NorthStar::updatePose() {
	int room = _robot->getRoom();
	int name = _robot->getName();

	// if we've changed rooms, prepare filters for this
	if (_lastRoom != -1 && _lastRoom != room) {

		LOG.write(LOG_MED, "NS_room_change", "Room change occurring.\n");

		// assume X and Y fir filters are of the same order
		int order = _filterX->getOrder(); 
		Pose *tempPose = new Pose(0.0, 0.0, 0.0);
		// adjust old filtered values according to new room
		for (int i = 0; i <= order; i++) {
			tempPose->reset(_oldX[i], _oldY[i], 0.0);
			
			tempPose->translate(-COL_OFFSET[0] - NS_ROOM_ORIGINS_FROM_COL[name][room][0], 
							    -COL_OFFSET[1] - NS_ROOM_ORIGINS_FROM_COL[name][room][1]);
			tempPose->scale(1.0/NS_ROOM_SCALE[name][room][0], 1.0/NS_ROOM_SCALE[name][room][1]);
			tempPose->rotate(-NS_ROOM_ROTATION[name][room]);
		
			_oldX[i] = tempPose->getX();
			_oldY[i] = tempPose->getY();
		}
		// use these updated values to seed the filters in preparation
		_filterX->seed(&_oldX);
		_filterY->seed(&_oldY);
		_filterTheta->seed(_robot->getInterface()->Theta());
	}

	_lastRoom = room;

	// get the newest filtered values from the robot
	float x = _getFilteredX();
	float y = _getFilteredY();
	float theta = _getFilteredTheta();

	LOG.write(LOG_LOW, "northStarUpdate", 
			  "north star (filtered) room %d: (%f, %f, %f)",
			  room+2, x, y, theta);

	// transform the data into global coord system
	Pose *estimate = new Pose(x, y, theta);
	if (room == ROOM_2) {
		// Apply specific linear transformation to Room 2, 
		// to correct for theta skew
		float xFitAngle = 0.0000204488 * x - 0.0804;
		float yFitAngle = 0.0000204488 * y - 0.0804;
		estimate->rotateEach(xFitAngle, yFitAngle, NS_ROOM_ROTATION[name][room]);
	}
	else {
		estimate->rotate(NS_ROOM_ROTATION[name][room]);
	}

	estimate->rotateEach(0, 0, THETA_SHIFT[name][room]);

	float sx = NS_ROOM_SCALE[name][room][0];
	float sy = NS_ROOM_SCALE[name][room][1];
	estimate->scale(sx, sy);

	float tx = COL_OFFSET[0] + NS_ROOM_ORIGINS_FROM_COL[name][room][0];
	float ty = COL_OFFSET[1] + NS_ROOM_ORIGINS_FROM_COL[name][room][1];
	estimate->translate(tx, ty);

	// update our pose with new global coords
	_pose->setX(estimate->getX());
	_pose->setY(estimate->getY());
	_pose->setTheta(estimate->getTheta());
	delete estimate;

	LOG.write(LOG_LOW, "northStarUpdate", 
			  "north star (pose) room %d: (%f, %f, %f)",
		      room+2, _pose->getX(), _pose->getY(), _pose->getTheta());

	// store the global x and y for future use
	_oldX.insert(_oldX.begin(), _pose->getX());
	_oldX.pop_back();
	_oldY.insert(_oldY.begin(), _pose->getY());
	_oldY.pop_back();
}

/**************************************
 * Definition: Returns filtered x from north star sensor
 *
 * Returns: filtered float coordinate
 **************************************/
float NorthStar::_getFilteredX() {
    int x = _robot->getInterface()->X();
    //return x;
    return _filterX->filter((float) x);
}

/**************************************
 * Definition: Returns filtered y from north star sensor
 *
 * Returns: filtered float coordinate
 **************************************/
float NorthStar::_getFilteredY() {
    int y = _robot->getInterface()->Y();
    //return y;
    return _filterY->filter((float) y);
}

/**************************************
 * Definition: Returns filtered theta from north star sensor
 *
 * Returns: filtered float theta
 **************************************/
float NorthStar::_getFilteredTheta() {
    float theta = _robot->getInterface()->Theta();
    return _filterTheta->filter(theta);
}
