#include "path.h"
#include "map.h"
#include <cstdio>

float NS_PENALTY[MAP_WIDTH][MAP_HEIGHT] = {
	// col 1 (starting from left of room)
	{0.80,
	 0.75,
	 0.50,
	 0.50,
	 0.50},
	// col 2
	{0.35,
	 1.00,
	 0.25,
	 1.00,
	 0.25},
	// col 3
	{0.10,
	 0.05,
	 0.01,
	 0.00,
	 0.00},
	// col 4
	{0.10,
	 1.00,
	 0.01,
	 1.00,
	 0.00},
	// col 5
	{0.10,
	 0.05,
	 0.01,
	 0.00,
	 0.00},
	// col 6
	{0.15,
	 1.00,
	 0.01,
	 1.00,
	 0.00},
	// col 7
	{0.40,
	 0.35,
	 0.01,
	 0.00,
	 0.00}
};

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

/** 
 * The value of a path is calculated as follows:
 * For each cell in the path, a NS penalty is applied, 
 * which knocks the points for that cell down by a 
 * percentage. Then, the amount of times the robot
 * must change directions is summed and subtracted
 * from the final path value.
**/
float Path::getValue() {
	float value = 0.0;

	bool used[MAP_WIDTH][MAP_HEIGHT] = {
		{false, false, false, false, false},
		{false, false, false, false, false},
		{false, false, false, false, false},
		{false, false, false, false, false},
		{false, false, false, false, false},
		{false, false, false, false, false},
		{false, false, false, false, false}
	};

	int directionChanges = 0;
	for (int i = 0; i < length(); i++) {
		Cell *nextCell = getCell(i);

		int nextX = nextCell->x;
		int nextY = nextCell->y;

		if (!used[nextX][nextY]) {
			int points = nextCell->getPoints();
			value += (float)points * (1-NS_PENALTY[nextX][nextY]);
			used[nextX][nextY] = true;
		}

		if (i > 1) {
			Cell *curCell = getCell(i-1);
			int curX = curCell->x;
			int curY = curCell->y;

			int heading = getHeading(i);
			switch (heading) {
			case DIR_NORTH:
				if (nextY < curY || nextX != curX) {
					directionChanges++;
				}
				break;
			case DIR_SOUTH:
				if (nextY > curY || nextX != curX) {
					directionChanges++;
				}
				break;
			case DIR_EAST:
				if (nextX > curX || nextY != curY) {
					directionChanges++;
				}
				break;
			case DIR_WEST:
				if (nextX < curX || nextY != curY) {
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
	return getCell(1);
}

Cell* Path::getLastCell() {
	return getCell(length()-1);
}

char* Path::toString() {
	char *str = (char *)malloc(length()*6);
	str[0] = '\0';
	for (int i = 1; i < length(); i++) {
		Cell *cell = getCell(i);
		char cellCoord[7];
		sprintf(cellCoord, "(%d,%d) ", cell->x, cell->y);
		strcat(str, cellCoord);
	}
	return str;
}
