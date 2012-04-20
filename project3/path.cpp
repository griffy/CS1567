#include "path.h"

Path::Path() 
: _cells() {	
}

Path::Path(Cell *cell) 
: _cells() {
	push(cell);
}

Path::Path(Path *path) 
: _cells() {
	for (int i = 0; i < path->length(); i++) {
		push(path->getCell(i));
	}
}

Path::~Path() {}

void Path::push(Cell *cell) {
	_cells.push_back(cell);
}

void Path::pop() {
	_cells.pop_back();
}

int Path::length() {
	return _cells.size();
}

int Path::getHeading(int upTo) {
	int heading = -1;

	if (upTo > 1) {
		Cell *curCell = getCell(upTo-1);
		Cell *prevCell = getCell(upTo-2);

		if (curCell->x != prevCell->x) {
			if (curCell->x > prevCell->x) {
				heading = DIR_WEST;
			}
			else {
				heading = DIR_EAST;
			}
		}
		else if (curCell->y != prevCell->y) {
			if (curCell->y > prevCell->y) {
				heading = DIR_NORTH;
			}
			else {
				heading = DIR_SOUTH;
			}
		}
	}

	return heading;
}

// The value of a path is calculated as follows:
// For each cell in the path, a NS penalty is applied, 
// which knocks the points for that cell down by a 
// percentage. Then, the amount of times the robot
// must change directions is summed and subtracted
// from the final path value.
float Path::getValue() {
	float value = 0.0;

	int directionChanges = 0;
	for (int i = 0; i < length(); i++) {
		Cell *nextCell = getCell(i);

		int points = nextCell->getPoints();
		value += (float)points * NS_PENALTY[nextCell->x][nextCell->y];

		if (i > 1) {
			Cell *curCell = getCell(i-1);
			int heading = getHeading(i);
			switch (heading) {
			case DIR_NORTH:
				if (nextCell->y < curCell->y ||
					nextCell->x != curCell->x) {
					directionChanges++;
				}
				break;
			case DIR_SOUTH:
				if (nextCell->y > curCell->y ||
					nextCell->x != curCell->x) {
					directionChanges++;
				}
				break;
			case DIR_EAST:
				if (nextCell->x > curCell->x ||
					nextCell->y != curCell->y) {
					directionChanges++;
				}
				break;
			case DIR_WEST:
				if (nextCell->x < curCell->x ||
					nextCell->y != curCell->y) {
					directionChanges++;
				}
				break;
			}
		}
	}

	value -= (float)directionChanges;

	return value;
}

Cell* Path::getCell(int i) {
	if (length() > i) {
		return _cells[i];
	}
	return NULL;
}

Cell* Path::getFirstCell() {
	return getCell(0);
}

Cell* Path::getLastCell() {
	return getCell(length()-1);
}
