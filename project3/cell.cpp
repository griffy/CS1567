#include "cell.h"


// we don't know what robot we are yet in the game 
int Cell::_robot = -1;

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

void Cell::setRobot() {
	if (_type == MAP_OBJ_ROBOT_1) {
		_robot = 1;
	}
	else {
		_robot = 2;
	}
}

/**************************************
 * Definition: Updates the cell's attributes according
 *             to the given map object
 *
 * Parameters: map_obj_t object at this cell's x and y
 **************************************/
void Cell::update(map_obj_t *mapObj) {
	// TODO: check this to make sure it's correct
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
			if (_robot == 1 && _type == MAP_OBJ_RESERVE_1) {
				return false;
			}
			else if (_robot == 2 && _type == MAP_OBJ_RESERVE_2) {
				return false;
			}
		}
		return true;
	}
	return false;
}

bool Cell::isPost() {
	return _post;
}

void Cell::setPost(bool post) {
	_post = post;
}

bool Cell::isOccupied() {
	return _occupied;
}

void Cell::setOccupied(bool occupied) {
	_occupied = occupied;
}

bool Cell::isReserved() {
	return _reserved;
}

void Cell::setReserved(bool reserved) {
	_reserved = reserved;
}

void Cell::addOpening(unsigned char direction) {
	_openings = _openings | direction;
}