#include "map.h"

Map::Map(RobotInterface *robotInterface) {
	_robotInterface = robotInterface;
	_score1 = 0;
	_score2 = 0;
	_loadMap();
}

Map::~Map() {}

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
	setOpenings(0,0);
}

int Map::getTeam1Score() {
	return _score1;
}

int Map::getTeam2Score() {
	return _score2;
}

bool Map::occupyCell(int x, int y) {
	return cells[x][y]->occupy(_robotInterface);
}

bool Map::reserveCell(int x, int y) {
	return cells[x][y]->reserve(_robotInterface);
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

void Map::setOpenings(int x, int y){
	if(x+1 < MAX_WIDTH){
		if(!cells[x+1][y]->isBlocked()){
			setOpenings(x+1,y);
			cells[x][y]->addOpening(1);
		}
	}
	if(x-1 > 0){
		if(!cells[x-1][y]->isBlocked()){
			setOpenings(x-1,y);
			cells[x][y]->addOpening(4);
		}
	}
	if(y+1 < MAX_HEIGHT){
		if(!cells[x][y+1]->isBlocked()){
			setOpenings(x,y+1);
			cells[x][y]->addOpening(2);
		}
	}
	if(y-1 > 0){
		if(!cells[x][y-1]->isBlocked()){
			setOpenings(x,y-1);
			cells[x][y]->addOpening(8);
		}
	}
}
