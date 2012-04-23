#include "map.h"
#include "logger.h"

Map::Map(RobotInterface *robotInterface, int startingX, int startingY) {
	_robotInterface = robotInterface;
	_score1 = 0;
	_score2 = 0;
	_loadMap();
	// we are the robot at this cell
	_claimRobotAt(startingX, startingY);
	_curCell = cells[startingX][startingY];
	LOG.write(LOG_LOW, "map", "starting cell: %d, %d",
			  startingX, startingY);
}

Map::~Map() {
	for (int x = 0; x < MAP_WIDTH; x++) {
		for (int y = 0; y < MAP_HEIGHT; y++) {
			delete cells[x][y];
		}
	}
}

void Map::update() {
	map_obj_t *map = _robotInterface->getMap(&_score1, &_score2);

	// iterate through the linked list map
	// and update each cell
	while (map != NULL) {
		int x = map->x;
		int y = map->y;

		cells[x][y]->update(map);

		map = map->next;
	}

	//_adjustOpenings();
}

int Map::getRobot1Score() {
	return _score1;
}

int Map::getRobot2Score() {
	return _score2;
}

Cell* Map::getCurrentCell() {
	return _curCell;
}

bool Map::canOccupy(int x, int y) {
	Cell *moveCell = NULL;
	moveCell = cellAt(x, y);
	if (moveCell == NULL) {
		return false;
	}
	return !moveCell->isBlocked();
}

Cell* Map::cellAt(int x, int y) {
	if (x < 0 || y < 0 || x >= MAP_WIDTH || y >= MAP_HEIGHT) {
		return NULL;
	}
	return cells[x][y];
}

bool Map::occupyCell(int x, int y) {
	if (cells[x][y]->occupy(_robotInterface)) {
		_curCell = cells[x][y];
		return true;
	}
	return false;
}

bool Map::reserveCell(int x, int y) {
	return cells[x][y]->reserve(_robotInterface);
}

void Map::_claimRobotAt(int x, int y) {
	cells[x][y]->claimRobot();
}

void Map::_loadMap() {
	// load the map to start with and fill in our
	// cell matrix
	map_obj_t *map = _robotInterface->getMap(&_score1, &_score2);

	// iterate through the linked list map
	while (map != NULL) {
		int x = map->x;
		int y = map->y;
		
		cells[x][y] = new Cell(map);

		map = map->next;
	}

	//_adjustOpenings();
}

void Map::_adjustOpenings(){
	for (int x = 0; x < MAP_WIDTH; x++) {
	    for (int y = 0; y < MAP_HEIGHT; y++) {    
		    if (x+1 < MAP_WIDTH) {
		        if (!cells[x+1][y]->isBlocked()) {
		        	cells[x][y]->addOpening(DIR_WEST);
		        }
			}

		    if (x-1 >= 0) {
		        if (!cells[x-1][y]->isBlocked()) {
		        	cells[x][y]->addOpening(DIR_EAST);
		        }
		    }

		    if (y+1 < MAP_HEIGHT) {
		        if (!cells[x][y+1]->isBlocked()) {
		        	cells[x][y]->addOpening(DIR_NORTH);
		        }
		    }

		    if (y-1 >= 0) {
		        if (!cells[x][y-1]->isBlocked()) {
		          	cells[x][y]->addOpening(DIR_SOUTH);
		        }
		    }
	    }
  	}
}
