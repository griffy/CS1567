/**
 * map.cpp
 * 
 * @brief 
 * 		This class represents the map given to us by the game server,
 *      and all operations on that map. It is self-updating, and
 *      has a public 2D array of cells for such things as path-finding.
 *
 * @author
 * 		Shawn Hanna
 * 		Tom Nason
 * 		Joel Griffith
 *
 **/

#include "map.h"
#include "logger.h"

Map::Map(RobotInterface *robotInterface, int startingX, int startingY) {
	_robotInterface = robotInterface;
	_score1 = 0;
	_score2 = 0;
	// load the map for the first time from the game server
	_loadMap();
	// we are the robot at this cell
	_claimRobotAt(startingX, startingY);
	_curCell = cells[startingX][startingY];
	LOG.write(LOG_LOW, "map", "starting cell: %d, %d",
			  startingX, startingY);
}

Map::~Map() {
	// free all the cells
	for (int x = 0; x < MAP_WIDTH; x++) {
		for (int y = 0; y < MAP_HEIGHT; y++) {
			delete cells[x][y];
		}
	}
}

/**************************************
 * Definition: Updates the map by querying the game server
 **************************************/
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
}

/**************************************
 * Definition: Returns Robot 1's score in the game
 *
 * Returns:    robot 1 score as int
 **************************************/
int Map::getRobot1Score() {
	return _score1;
}

/**************************************
 * Definition: Returns Robot 2's score in the game
 *
 * Returns:    robot 2 score as int
 **************************************/
int Map::getRobot2Score() {
	return _score2;
}

/**************************************
 * Definition: Returns the cell our robot is currently at
 *
 * Returns:    pointer to the cell
 **************************************/
Cell* Map::getCurrentCell() {
	return _curCell;
}

/**************************************
 * Definition: Whether or not we can occupy the cell at x,y
 *
 * Parameters: x and y as ints
 *
 * Returns:    true if we can, false if not
 **************************************/
bool Map::canOccupy(int x, int y) {
	Cell *moveCell = NULL;
	moveCell = cellAt(x, y);
	if (moveCell == NULL) {
		return false;
	}
	return !moveCell->isBlocked();
}

/**************************************
 * Definition: Returns the cell at x,y
 *
 * Parameters: x and y as ints
 *
 * Returns:    pointer to cell, or NULL if doesn't exist
 **************************************/
Cell* Map::cellAt(int x, int y) {
	if (x < 0 || y < 0 || x >= MAP_WIDTH || y >= MAP_HEIGHT) {
		return NULL;
	}
	return cells[x][y];
}

/**************************************
 * Definition: Attempts to occupy the cell at x,y by talking to
 *             the game server
 *
 * Parameters: x and y as ints
 *
 * Returns:    true if successful, false if not
 **************************************/
bool Map::occupyCell(int x, int y) {
	if (cells[x][y]->occupy(_robotInterface)) {
		_curCell = cells[x][y];
		return true;
	}
	return false;
}

/**************************************
 * Definition: Attempts to reserve the cell at x,y by talking to
 *             the game server
 *
 * Parameters: x and y as ints
 *
 * Returns:    true if successful, false if not
 **************************************/
bool Map::reserveCell(int x, int y) {
	return cells[x][y]->reserve(_robotInterface);
}

/**************************************
 * Definition: Tells the cell at x,y that this is our robot
 *
 * Parameters: x and y as ints
 **************************************/
void Map::_claimRobotAt(int x, int y) {
	cells[x][y]->claimRobot();
}

/**************************************
 * Definition: Loads the map for the first time from the game server,
 *             creating our 2D array of cells
 **************************************/
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
}