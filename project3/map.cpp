#include "map.h"

Map::Map(RobotInterface *robotInterface, int startingX, int startingY) {
	_robotInterface = robotInterface;
	_score1 = 0;
	_score2 = 0;
	_loadMap();
	// we are the robot at this cell
	_setRobotAt(startingX, startingY);
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
}

int Map::getRobot1Score() {
	return _score1;
}

int Map::getRobot2Score() {
	return _score2;
}

bool Map::occupyCell(int x, int y) {
	return cells[x][y]->occupy(_robotInterface);
}

bool Map::reserveCell(int x, int y) {
	return cells[x][y]->reserve(_robotInterface);
}

void Map::_setRobotAt(int x, int y) {

	cells[x][y]->setRobot();
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
}

