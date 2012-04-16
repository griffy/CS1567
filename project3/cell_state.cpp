#include "cell_state.h"

CellState::CellState(int cellType) {
	_state = STATE_STAND;
	_tagState = TAGS_BOTH_GE_TWO;
	_cellType = cellType;
}

CellState::~CellState() {}

int CellState::update(int newTagState) {
	int newState;

	switch (_cellType) {
	case CELL_T:
		newState = _updateCellT(newTagState);
		break;
	case CELL_CORNER:
		newState = _updateCellCorner(newTagState);
		break;
	case CELL_HALL:
		newState = _updateCellHall(newTagState);
		break;
	case CELL_INTERSECTION:
		newState = _updateCellIntersection(newTagState);
		break;
	}

	return newState;
}

int CellState::_updateCellT(int newTagState) {
	int newState;

	switch (_tagState) {
	case TAGS_BOTH_GE_TWO:
		switch (newTagState) {
		case TAGS_BOTH_GE_TWO:
			newState = STATE_STAND;
			break;
		case TAGS_BOTH_ONE:
			newState = STATE_TURN;
			break;
		case TAGS_ONE_OR_NONE:
			newState = STATE_TURN;
			break;
		case TAGS_LESS_LEFT:
			newState = STATE_STRAFE;
			break;
		case TAGS_LESS_RIGHT:
			newState = STATE_STRAFE;
			break;
		}
		break;
	case TAGS_BOTH_ONE:
		switch (newTagState) {
		case TAGS_BOTH_GE_TWO:
			break;
		case TAGS_BOTH_ONE:
			break;
		case TAGS_ONE_OR_NONE:
			break;
		case TAGS_LESS_LEFT:
			break;
		case TAGS_LESS_RIGHT:
			break;
		}
		break;
	case TAGS_ONE_OR_NONE:
		switch (newTagState) {
		case TAGS_BOTH_GE_TWO:
			break;
		case TAGS_BOTH_ONE:
			break;
		case TAGS_ONE_OR_NONE:
			break;
		case TAGS_LESS_LEFT:
			break;
		case TAGS_LESS_RIGHT:
			break;
		}
		break;
	case TAGS_LESS_LEFT:
		switch (newTagState) {
		case TAGS_BOTH_GE_TWO:
			break;
		case TAGS_BOTH_ONE:
			break;
		case TAGS_ONE_OR_NONE:
			break;
		case TAGS_LESS_LEFT:
			break;
		case TAGS_LESS_RIGHT:
			break;
		}
		break;
	case TAGS_LESS_RIGHT:
		switch (newTagState) {
		case TAGS_BOTH_GE_TWO:
			break;
		case TAGS_BOTH_ONE:
			break;
		case TAGS_ONE_OR_NONE:
			break;
		case TAGS_LESS_LEFT:
			break;
		case TAGS_LESS_RIGHT:
			break;
		}
		break;
	}

	_state = newState;
	_tagState = newTagState;

	return newState;
}

int CellState::_updateCellCorner(int newTagState) {
	int newState;

	switch (_tagState) {
	case TAGS_BOTH_GE_TWO:
		switch (newTagState) {
		case TAGS_BOTH_GE_TWO:
			newState = STATE_STAND; // should be stand?
			break;
		case TAGS_BOTH_ONE:
			newState = STATE_TURN;
			break;
		case TAGS_ONE_OR_NONE:
			newState = STATE_TURN;
			break;
		case TAGS_LESS_LEFT:
			newState = STATE_STRAFE;
			break;
		case TAGS_LESS_RIGHT:
			newState = STATE_STRAFE;
			break;
		}
		break;
	case TAGS_BOTH_ONE:
		switch (newTagState) {
		case TAGS_BOTH_GE_TWO:
			break;
		case TAGS_BOTH_ONE:
			break;
		case TAGS_ONE_OR_NONE:
			break;
		case TAGS_LESS_LEFT:
			break;
		case TAGS_LESS_RIGHT:
			break;
		}
		break;
	case TAGS_ONE_OR_NONE:
		switch (newTagState) {
		case TAGS_BOTH_GE_TWO:
			break;
		case TAGS_BOTH_ONE:
			break;
		case TAGS_ONE_OR_NONE:
			break;
		case TAGS_LESS_LEFT:
			break;
		case TAGS_LESS_RIGHT:
			break;
		}
		break;
	case TAGS_LESS_LEFT:
		switch (newTagState) {
		case TAGS_BOTH_GE_TWO:
			break;
		case TAGS_BOTH_ONE:
			break;
		case TAGS_ONE_OR_NONE:
			break;
		case TAGS_LESS_LEFT:
			break;
		case TAGS_LESS_RIGHT:
			break;
		}
		break;
	case TAGS_LESS_RIGHT:
		switch (newTagState) {
		case TAGS_BOTH_GE_TWO:
			break;
		case TAGS_BOTH_ONE:
			break;
		case TAGS_ONE_OR_NONE:
			break;
		case TAGS_LESS_LEFT:
			break;
		case TAGS_LESS_RIGHT:
			break;
		}
		break;
	}

	_state = newState;
	_tagState = newTagState;

	return newState;
}

int CellState::_updateCellHall(int newTagState) {
	int newState;

	switch (_tagState) {
	case TAGS_BOTH_GE_TWO:
		switch (newTagState) {
		case TAGS_BOTH_GE_TWO:
			newState = STATE_TURN; // should be stand?
			break;
		case TAGS_BOTH_ONE:
			newState = STATE_TURN;
			break;
		case TAGS_ONE_OR_NONE:
			newState = STATE_TURN;
			break;
		case TAGS_LESS_LEFT:
			newState = STATE_STRAFE;
			break;
		case TAGS_LESS_RIGHT:
			newState = STATE_STRAFE;
			break;
		}
		break;
	case TAGS_BOTH_ONE:
		switch (newTagState) {
		case TAGS_BOTH_GE_TWO:
			break;
		case TAGS_BOTH_ONE:
			break;
		case TAGS_ONE_OR_NONE:
			break;
		case TAGS_LESS_LEFT:
			break;
		case TAGS_LESS_RIGHT:
			break;
		}
		break;
	case TAGS_ONE_OR_NONE:
		switch (newTagState) {
		case TAGS_BOTH_GE_TWO:
			break;
		case TAGS_BOTH_ONE:
			break;
		case TAGS_ONE_OR_NONE:
			break;
		case TAGS_LESS_LEFT:
			break;
		case TAGS_LESS_RIGHT:
			break;
		}
		break;
	case TAGS_LESS_LEFT:
		switch (newTagState) {
		case TAGS_BOTH_GE_TWO:
			break;
		case TAGS_BOTH_ONE:
			break;
		case TAGS_ONE_OR_NONE:
			break;
		case TAGS_LESS_LEFT:
			break;
		case TAGS_LESS_RIGHT:
			break;
		}
		break;
	case TAGS_LESS_RIGHT:
		switch (newTagState) {
		case TAGS_BOTH_GE_TWO:
			break;
		case TAGS_BOTH_ONE:
			break;
		case TAGS_ONE_OR_NONE:
			break;
		case TAGS_LESS_LEFT:
			break;
		case TAGS_LESS_RIGHT:
			break;
		}
		break;
	}

	_state = newState;
	_tagState = newTagState;

	return newState;
}

int CellState::_updateCellIntersection(int newTagState) {
	int newState;

	switch (_tagState) {
	case TAGS_BOTH_GE_TWO:
		switch (newTagState) {
		case TAGS_BOTH_GE_TWO:
			newState = STATE_TURN; // should be stand?
			break;
		case TAGS_BOTH_ONE:
			newState = STATE_TURN;
			break;
		case TAGS_ONE_OR_NONE:
			newState = STATE_TURN;
			break;
		case TAGS_LESS_LEFT:
			newState = STATE_STRAFE;
			break;
		case TAGS_LESS_RIGHT:
			newState = STATE_STRAFE;
			break;
		}
		break;
	case TAGS_BOTH_ONE:
		switch (newTagState) {
		case TAGS_BOTH_GE_TWO:
			break;
		case TAGS_BOTH_ONE:
			break;
		case TAGS_ONE_OR_NONE:
			break;
		case TAGS_LESS_LEFT:
			break;
		case TAGS_LESS_RIGHT:
			break;
		}
		break;
	case TAGS_ONE_OR_NONE:
		switch (newTagState) {
		case TAGS_BOTH_GE_TWO:
			break;
		case TAGS_BOTH_ONE:
			break;
		case TAGS_ONE_OR_NONE:
			break;
		case TAGS_LESS_LEFT:
			break;
		case TAGS_LESS_RIGHT:
			break;
		}
		break;
	case TAGS_LESS_LEFT:
		switch (newTagState) {
		case TAGS_BOTH_GE_TWO:
			break;
		case TAGS_BOTH_ONE:
			break;
		case TAGS_ONE_OR_NONE:
			break;
		case TAGS_LESS_LEFT:
			break;
		case TAGS_LESS_RIGHT:
			break;
		}
		break;
	case TAGS_LESS_RIGHT:
		switch (newTagState) {
		case TAGS_BOTH_GE_TWO:
			break;
		case TAGS_BOTH_ONE:
			break;
		case TAGS_ONE_OR_NONE:
			break;
		case TAGS_LESS_LEFT:
			break;
		case TAGS_LESS_RIGHT:
			break;
		}
		break;
	}

	_state = newState;
	_tagState = newTagState;

	return newState;
}