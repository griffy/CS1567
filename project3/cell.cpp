#include "cell.h"

Cell::Cell(map_obj_t *mapObj) {
	x = mapObj->x;
	y = mapObj->y;

	setPoints(mapObj->points);

	setOccupied(false);
	if (mapObj->type == MAP_OBJ_ROBOT_1 ||
		mapObj->type == MAP_OBJ_ROBOT_2) {
		setOccupied(true);
	}
	setReserved(false);
	if (mapObj->type == MAP_OBJ_RESERVE_1 ||
		mapObj->type == MAP_OBJ_RESERVE_2) {
		setReserved(true);
	}
}

Cell::~Cell() {}

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

int Cell::getPoints() {
	return _points;
}

void Cell::setPoints(int points) {
	_points = points;
}

/* Definition: Catch-all method that says it's blocked if
   			   the cell is a post, a robot is in it, or it is
   			   reserved
 */
bool Cell::isBlocked() {
	return _blocked;
}

void Cell::setBlocked(bool blocked) {
	_blocked = blocked;
}

bool Cell::isPost() {
	return _post;
}

void Cell::setPost(bool post) {
	_post = post;
	_updateBlocked();
}

bool Cell::isOccupied() {
	return _occupied;
}

void Cell::setOccupied(bool occupied) {
	_occupied = occupied;
	_updateBlocked();
}

bool Cell::isReserved() {
	return _reserved;
}

void Cell::setReserved(bool reserved) {
	_reserved = reserved;
	_updateBlocked();
}

void Cell::_updateBlocked() {
	setBlocked(false);
	if (isPost() || isReserved() || isOccupied()) {
		setBlocked(true);
	}
}