#include "map_strategy.h"
#include "logger.h"

MapStrategy::MapStrategy(Map *map) {
	_map = map;
}

MapStrategy::~MapStrategy() {
	clearPaths();
}

Cell* MapStrategy::nextCell() {
	_map->update();

	Path *path = getBestPath(PATH_LENGTH);
	if (path == NULL || path->getValue() == 0) {
		path = getBestPath(10);
		if (path == NULL || path->getValue() == 0) {
			return NULL;
		}
	}
	
	LOG.write(LOG_LOW, "nextCell",
		      "best path value: %f", path->getValue());
	LOG.write(LOG_LOW, "nextCell", 
			  "best path: %s", path->toString());

	return path->getFirstCell();
}

Path* MapStrategy::getBestPath(int length) {
	Path *bestPath = NULL;
	Path *curPath = new Path(_map->getCurrentCell());

	createPaths(curPath, length);

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
	clearPaths();
	
	return bestPath;
}

void MapStrategy::createPaths(Path *parentPath, int length) {
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
			createPaths(curPath, length-1);
			curPath->pop();
		}
	}

	delete curPath;
}

void MapStrategy::clearPaths() {
	for (int i = 0; i < _pathList.size(); i++) {
		delete _pathList[i];
	}
	_pathList.clear();
}
