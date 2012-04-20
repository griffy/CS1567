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
	_cells->push(cell);
}

void Path::pop() {
	_cells->pop();
}

int Path::length() {
	return _cells->size();
}

int Path::getHeading(int upTo) {
	if (upTo > 1) {
		Cell *curCell = getCell(upTo-1);
		Cell *prevCell = getCell(upTo-2);

		if (curCell->x != prevCell->x) {
			if (
		}
	}
}

double Path::getValue() {
	double value = 0.0;

	int directionChanges = 0;
	for (int i = 0; i < length(); i++) {
		Cell *nextCell = getCell(i);
		value += cell->getPoints();
		if (i > 1) {
			Cell *curCell = getCell(i-1);
			Cell *prevCell = getCell(i-2);
			if (cell->x != prevCell->x && cell->y != prevCell->y) {
				directionChanges++;
			}
		}
	}

	value -= (float)directionChanges;
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
