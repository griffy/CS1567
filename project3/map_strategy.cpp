/**
 * map_strategy.cpp
 * 
 * @brief 
 *      This class is used to implement path-finding for an instance
 *      of the Map class. 
 *
 * @author
 * 		Shawn Hanna
 * 		Tom Nason
 * 		Joel Griffith
 *
 **/

#include "map_strategy.h"
#include "logger.h"

MapStrategy::MapStrategy(Map *map) {
	_map = map;
}

MapStrategy::~MapStrategy() {
	_clearPaths();
}

/**************************************
 * Definition: Returns the next cell that should be occupied by
 *             our robot in the game
 *
 * Returns:    pointer to cell
 **************************************/
Cell* MapStrategy::nextCell() {
	_map->update();

	Path *path = _getBestPath(PATH_LENGTH);
	if (path == NULL || path->getValue() == 0) {
		path = _getBestPath(10);
		if (path == NULL || path->getValue() == 0) {
			return NULL;
		}
	}

	return path->getFirstCell();
}

/**************************************
 * Definition: Returns the best path possible from this point in
 *             the game
 *
 * Parameters: length of path as int (search depth)
 *
 * Returns:    pointer to Path
 **************************************/
Path* MapStrategy::_getBestPath(int length) {
	Path *bestPath = NULL;
	Path *curPath = new Path(_map->getCurrentCell());

	_createPaths(curPath, length);

	float bestPathVal = -10000.0;
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
	_clearPaths();
	
	return bestPath;
}

/**************************************
 * Definition: Creates a list of all possible paths branching from
 *             the parent path
 *
 * Parameters: pointer to parent path and length of path as int (search depth)
 **************************************/
void MapStrategy::_createPaths(Path *parentPath, int length) {
	Path *curPath = new Path(parentPath);

	if (length == 0) {
		_pathList.push_back(curPath);
		return;
	}

	int x = curPath->getLastCell()->x;
	int y = curPath->getLastCell()->y;

	int newX[4] = {x, x, x-1, x+1};
	int newY[4] = {y+1, y-1, y, y};

	for (int i = 0; i < 4; i++) {
		if (_map->canOccupy(newX[i], newY[i])) {
			curPath->push(_map->cells[newX[i]][newY[i]]);
			_createPaths(curPath, length-1);
			curPath->pop();
		}
	}

	delete curPath;
}

/**************************************
 * Definition: Clears the current list of paths
 **************************************/
void MapStrategy::_clearPaths() {
	for (int i = 0; i < _pathList.size(); i++) {
		delete _pathList[i];
	}
	_pathList.clear();
}
