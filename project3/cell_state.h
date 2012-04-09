#ifndef CS1567_CELLSTATE_H
#define CS1567_CELLSTATE_H

#include "cell.h"

#define STATE_TURN 0
#define STATE_STRAFE 1
#define STATE_STAND 2

class CellState {
public:
	CellState(int cellType);
	~CellState();
	int update(int newTagState);
private:
	int _updateCellT(newTagState);
	int _updateCellCorner(newTagState);
	int _updateCellHall(newTagState);
	int _updateCellIntersection(newTagState);

	int _state;
	int _tagState;
	int _cellType;
};

#endif