#include "map_strategy.h"

MapStrategy::MapStrategy(Map *map) {
	_map = map;
}

MapStrategy::~MapStrategy() {}

/*
int Map::getRobot1Score();
int Map::getRobot2Score();
Cell* Map::getCurrentCell();
bool Map::occupyCell(int x, int y);
bool Map::reserveCell(int x, int y);
*/

Cell* MapStrategy::nextCell() {
	_map->update();
	
	Path *path = getBestPath(PATH_LENGTH);
	if (path == NULL || path->getValue() == 0) {
		path = getBestPath(10);
		if (path == NULL || path->getValue() == 0) {
			return NULL;
		}
	}
	
	return path->getFirstCell();
}

/*
// TODO: use MiniMax algorithm
Cell* MapStrategy::nextCell() {
	_map->update();

	Cell *curCell = _map->getCurrentCell();
	int x = curCell->x;
	int y = curCell->y;

	int openings = curCell->getOpenings();
	int best = 0;
	int adder=1;

	int direction=0;
	while(1){
		for(int i=0; i<3; i++){
			if(y+adder-1<MAP_HEIGHT) {
				openings = _map->cells[x][y+adder-1]->getOpenings();
				if(i == 0 && (openings & DIR_NORTH) && direction != DIR_NORTH){
					if(_map->cells[x][y+adder]->getPoints() > best) {
						best = _map->cells[x][y+adder]->getPoints();
						direction=DIR_NORTH;
					}
				}
			}
			if(x-adder+1>0){
				openings = _map->cells[x-adder+1][y]->getOpenings();
				if(i == 1 && (openings & DIR_EAST) && direction != DIR_NORTH){
					if(_map->cells[x-adder][y]->getPoints() > best) {
						best = _map->cells[x-adder][y]->getPoints();
						direction=DIR_EAST;
					}
				}
			}
			// check west
			if(x+adder-1<MAP_WIDTH){
				openings = _map->cells[x+adder-1][y]->getOpenings();
				if(i == 2 && (openings & DIR_WEST) && direction != DIR_NORTH){
					if(_map->cells[x-adder][y]->getPoints() > best){
						best = _map->cells[x+adder][y]->getPoints();
						direction=DIR_WEST;
					}
				}
			}
			// check south
			if(y-adder+1>0) {
				openings = _map->cells[x][y-adder+1]->getOpenings();
				if(i == 3 && (openings & DIR_SOUTH) && direction != DIR_NORTH){
					if(_map->cells[x][y-adder]->getPoints() > best){
						best = _map->cells[x][y-adder]->getPoints();
						direction=DIR_SOUTH;
					}
				}
			}
		}
		if(direction == 0){
			adder++;
		}
		else{
			break;
		}
		if(adder>MAP_WIDTH){
			break;
		}
	}

	switch (direction) {
		case DIR_NORTH:
			// if this is false, panic and slam into a wall?
			_map->reserveCell(x,y+1);
			return _map->cells[x][y+1];
		case DIR_SOUTH:
			_map->reserveCell(x,y-1);
			return _map->cells[x][y-1];
		case DIR_EAST:
			_map->reserveCell(x-1,y);
			return _map->cells[x-1][y];
		case DIR_WEST:
			_map->reserveCell(x+1,y);
			return _map->cells[x+1][y];
	}
	
	return NULL;
}
*/

/********************************************
 * Definition:
 * 
 *******************************************/
Path* MapStrategy::getBestPath(int length) {
	Path *bestPath = NULL;
	Path *curPath = new Path(_map->getCurrentCell());
	createPaths(curPath, length);
	float bestPathVal = 0.0;
	int bestPathIndex = -1;
	for (int i = 0; i < _pathList.size(); i++) {
		float pathVal = _pathList[i]->getValue();
		if (pathVal > bestPathVal) {
			bestPathVal = pathVal;
			bestPathIndex = i;
		}
	}
	
	if (bestPathIndex != -1) {
		bestPath = _pathList[bestPathIndex];
		_pathList.erase(_pathList.begin()+bestPathIndex);
	}
	clearPaths();
	return bestPath;
}

void MapStrategy::createPaths(Path *parentPath, int length) {
	Path *curPath = new Path(parentPath);

	if (length == 0) {
		_pathList.push_back(curPath);
		return;
	}

	int openings = curPath->getLastCell()->getOpenings();
	if (openings > 0) {
		int x = curPath->getLastCell()->x;
		int y = curPath->getLastCell()->y;

		int newX[4] = {x, x, x-1, x+1};
		int newY[4] = {y+1, y-1, y, y};

		for (int i = 0; i < 4; i++) {
			if (_map->canOccupy(newX[i], newY[i])) {
				curPath->push(_map->cells[newX[i]][newY[i]]);
				createPaths(curPath, length-1);
				curPath->pop();
			}
		}
	}
}

void MapStrategy::clearPaths() {
	for (int i = 0; i < _pathList.size(); i++) {
		delete _pathList[i];
	}
	_pathList.clear();
}

/*
// MiniMax notes:
// Branching factor will be ~4, and
// the nodes we'd generate would be 4^n
// So, we should probably limit depth to
// 8 or so if we want to be performant
Cell* MapStrategy::nextCell() {
	_map->update();

	Cell *curCell = _map->getCurrentCell();

	// create a 2d array representing the map
	// with all the points that are left.
	// 0 is a cell without points
	// > 0 is a cell with points
	// -3 is an obstacle
	// -2 is the other robot
	// -1 is our robot
	int mapValues[MAP_WIDTH][MAP_HEIGHT];
	for (int x = 0; x < MAP_WIDTH; x++) {
		for (int y = 0; y < MAP_HEIGHT; y++) {
			if (_map->canOccupy(x, y)) {
				mapValues[x][y] = _map->cellAt(x, y)->getPoints();
			}
			else {
				if (_map->robotAt(x, y) == 1) {
					if (Cell::robot == 1) {
						mapValues[x][y] = -1;
					}
					else {
						mapValues[x][y] = -2;
					}
				}
				else if (_map->robotAt(x, y) == 2) {
					if (Cell::robot == 2) {
						mapValues[x][y] = -1;
					}
					else {
						mapValues[x][y] = -2;
					}
				}
				else {	
					mapValues[x][y] = -3;
				}
			}
		}
	}

	Move *initial = new Move;
	initial->robot = -1;
	initial->x = curCell->x;
	initial->y = curCell->y;
	initial->next = NULL;
	initial->value = 0;

	Move *move = maxMove(mapValues, initial, 0, 0, 0);
	if (move == NULL) {
		return NULL;
	}
	
	switch (move->direction) {
	case DIR_NORTH:
		return _map->cellAt(curCell->x, curCell->y+1);
	case DIR_SOUTH:
		return _map->cellAt(curCell->x, curCell->y-1);
	case DIR_EAST:
		return _map->cellAt(curCell->x-1, curCell->y);
	case DIR_WEST:
		return _map->cellAt(curCell->x+1, curCell->y);
	}
}

void MapStrategy::apply(Move *move, int map[][]) {
	Move *curMove = move;
	while (curMove != NULL) {
		map[curMove->x][curMove->y] = 0;
	}
}

Move* MapStrategy::maxMove(bool visited[][], Move *move, int depth, int alpha, int beta) {
	if (depth > MAX_SEARCH_DEPTH) {
		return move;
	}
	else {
		Move *best = NULL;
		apply(move, map);
	}
}

/*
// TODO: Each Move needs to have its next and prev
//       links set so a Move is a list of Moves
Move* MapStrategy::maxMove(Cell *cur, Move *best, int depth, int alpha, int beta) {
	if (depth > MAX_SEARCH_DEPTH || cur == NULL) {
		return best;
	}

	int x = cur->x;
	int y = cur->y;

	int newX[4] = {x, x, x-1, x+1};
	int newY[4] = {y+1, y-1, y, y};

	for (int i = 0; i < 4; i++) {
		if (canOccupy(newX[i], newY[i])) {
			Move *move = minMove(_map->cellAt(newX[i], newY[i]), 
								 best, depth+1, alpha, beta);
			if (move == NULL) {

			}


			if (best == NULL) {
				best = move;
			}
			else {
				if (move->cost > best->cost) {
					best = move;
					alpha = move->cost;
				}
			}

			if (beta > alpha) {
				return best;
			}
		}
	}

	return best;
}

// TODO: minMove
*/