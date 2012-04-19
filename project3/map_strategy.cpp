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
/*
// MiniMax notes:
// Branching factor will be ~4, and
// the nodes we'd generate would be 4^n
// So, we should probably limit depth to
// 8 or so if we want to be performant
Cell* MapStrategy::nextCell() {
	_map->update();

	Cell *curCell = _map->getCurrentCell();
	return maxMove(curCell, NULL, 0, 0, 0);
}

// TODO: Each Move needs to have its next and prev
//       links set so a Move is a list of Moves
Move* MapStrategy::maxMove(Cell *cur, Move *best, int depth, int alpha, int beta) {
	if (depth > MAX_SEARCH_DEPTH || cur == NULL) {
		return best;
	}

	int x = cur->x;
	int y = cur->y;

	Move *bestMove = NULL;

	int newX[4] = {x, x, x-1, x+1};
	int newY[4] = {y+1, y-1, y, y};

	for (int i = 0; i < 4; i++) {
		if (canOccupy(newX[i], newY[i])) {
			Move *move = minMove(_map->cellAt(newX[i], newY[i]), best, depth+1, alpha, beta);
			if (bestMove == NULL) {
				bestMove = move;
			}
			else {
				if (move->cost > bestMove->cost) {
					bestMove = move;
					alpha = move->cost;
				}
			}

			if (beta > alpha) {
				return bestMove;
			}
		}
	}

	return bestMove;
}

// TODO: minMove

*/