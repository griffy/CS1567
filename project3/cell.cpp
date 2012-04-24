/**
 * cell.cpp
 * 
 * @brief 
 * 		This class is used to represent a single cell in a grid, and
 *      all the operations on that cell (including updates from the
 *      game server)
 * 
 * @author
 * 		Shawn Hanna
 * 		Tom Nason
 * 		Joel Griffith
 * 
 **/

#include "cell.h"

// we don't know what robot we are yet in the game 
int Cell::robot = -1;

Cell::Cell(map_obj_t *mapObj) {
	x = mapObj->x;
	y = mapObj->y;

	_type = mapObj->type;

	setPoints(mapObj->points);
	setOccupied(false);
	setReserved(false);
	setPost(false);
	switch (_type) {
	case MAP_OBJ_EMPTY:
		setPoints(0);
		break;
	case MAP_OBJ_ROBOT_1:
	case MAP_OBJ_ROBOT_2:
		setOccupied(true);
		break;
	case MAP_OBJ_RESERVE_1:
	case MAP_OBJ_RESERVE_2:
		setReserved(true);
		break;
	case MAP_OBJ_POST:
		setPost(true);
		break;
	}
}

Cell::~Cell() {}

/**************************************
 * Definition: When called, sets the static variable we
 *             use to determine what robot we are to be
 *             the robot at this cell
 **************************************/
void Cell::claimRobot() {
	if (_type == MAP_OBJ_ROBOT_1) {
		robot = 1;
	}
	else if (_type == MAP_OBJ_ROBOT_2) {
		robot = 2;
	}
}

/**************************************
 * Definition: Updates the cell's attributes according
 *             to the given map object
 *
 * Parameters: map_obj_t object at this cell's x and y
 **************************************/
void Cell::update(map_obj_t *mapObj) {
	switch (mapObj->type) {
	case MAP_OBJ_EMPTY:
		setPoints(0);
		break;
	case MAP_OBJ_ROBOT_1:
	case MAP_OBJ_ROBOT_2:
		if (!isOccupied()) {
			setOccupied(true);
		}
		break;
	case MAP_OBJ_PELLET:
		if (mapObj->points != getPoints()) {
			setPoints(mapObj->points);
		}
		break;
	case MAP_OBJ_RESERVE_1:
	case MAP_OBJ_RESERVE_2:
		if (!isReserved()) {
			setReserved(true);
		}
		break;
	}
}

/**************************************
 * Definition: Attempts to occupy a cell in the game
 *
 * Parameters: a RobotInterface instance to communicate
 *             with game server
 *
 * Returns:    returns true on success and false on failure
 **************************************/
bool Cell::occupy(RobotInterface *robotInterface) {
	if (!isOccupied()) {
		if (robotInterface->updateMap(x, y) == RI_RESP_SUCCESS) {
			setOccupied(true);
			return true;
		}
	}
	return false;
}

/**************************************
 * Definition: Attempts to reserve a cell in the game
 *
 * Parameters: a RobotInterface instance to communicate
 *             with game server
 *
 * Returns:    returns true on success and false on failure
 **************************************/
bool Cell::reserve(RobotInterface *robotInterface) {
	if (!isReserved()) {
		if (robotInterface->reserveMap(x, y) == RI_RESP_SUCCESS) {
			setReserved(true);
			return true;
		}
	}
	return false;
}

/**************************************
 * Definition: Returns the amount of points the cell is
 *             currently worth
 *
 * Returns:    points as an integer
 **************************************/
int Cell::getPoints() {
	return _points;
}

/**************************************
 * Definition: Sets the new points value
 *
 * Parameters: the new integer points value
 **************************************/
void Cell::setPoints(int points) {
	_points = points;
}

/**************************************
 * Definition: Determines if the cell is blocked.
 *             A cell can be blocked by being:
 *              - a post
 *              - reserved (by other robot)
 *              - occupied
 *
 * Returns:    returns true on success and false on failure
 **************************************/
bool Cell::isBlocked() {
	if (isPost() || isOccupied()) {
		if (isReserved()) {
			if ((robot == 1 && _type == MAP_OBJ_RESERVE_1) ||
			    (robot == 2 && _type == MAP_OBJ_RESERVE_2)) {
				return false;
			}
		}
		return true;
	}
	return false;
}

/**************************************
 * Definition: Returns whether or not this cell is a post
 *
 * Returns:    true if post, false otherwise
 **************************************/
bool Cell::isPost() {
	return _post;
}

/**************************************
 * Definition: Sets this cell's post attribute
 *
 * Parameters: bool specifying if post or not
 **************************************/
void Cell::setPost(bool post) {
	_post = post;
}

/**************************************
 * Definition: Returns whether or not this cell is occupied
 *
 * Returns:    true if occupied, false otherwise
 **************************************/
bool Cell::isOccupied() {
	return _occupied;
}

/**************************************
 * Definition: Sets this cell's occupied attribute
 *
 * Parameters: bool specifying if occupied or not
 **************************************/
void Cell::setOccupied(bool occupied) {
	_occupied = occupied;
}

/**************************************
 * Definition: Returns whether or not this cell is reserved
 *
 * Returns:    true if reserved, false otherwise
 **************************************/
bool Cell::isReserved() {
	return _reserved;
}

/**************************************
 * Definition: Sets this cell's reserved attribute
 *
 * Parameters: bool specifying if reserved or not
 **************************************/
void Cell::setReserved(bool reserved) {
	_reserved = reserved;
}