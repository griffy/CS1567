#ifndef CS1567_CELLSTATE_H
#define CS1567_CELLSTATE_H

#include "cell.h"

#define TAGS_BOTH_GE_TWO 0 // >= 2 tags on both sides
#define TAGS_BOTH_ONE 1 // 1 tag on both sides
#define TAGS_ONE_OR_NONE 2 // 1 or no tags
#define TAGS_LESS_LEFT 3 // less tags on left than right
#define TAGS_LESS_RIGHT 4 // less tags on right than left

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