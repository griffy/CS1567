#include "map.h"
#include "logger.h"

Map::Map(RobotInterface *robotInterface, int startingX, int startingY) {
	_robotInterface = robotInterface;
	_score1 = 0;
	_score2 = 0;
	_loadMap();
	// we are the robot at this cell
	_setRobotAt(startingX, startingY);
	_curCell = cells[startingX][startingY];
	LOG.write(LOG_LOW, "map", "starting cell: %d, %d",
			  startingX, startingY);
        reserveCell(startingX, startingY);
        occupyCell(startingX, startingY);
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
	_adjustOpenings();
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

bool Map::occupyCell(int x, int y) {
	if (cells[x][y]->occupy(_robotInterface)) {
		_curCell->setOccupied(false);
		_curCell->setReserved(false);
		_curCell = cells[x][y];
		return true;
	}
	return false;
}

bool Map::reserveCell(int x, int y) {
	return cells[x][y]->reserve(_robotInterface);
}

void Map::_setRobotAt(int x, int y) {
	//NOTE: do we need to 'unset' a different cell?
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
	_adjustOpenings();
        for (int x=0; x<MAP_WIDTH; x++){
                for(int y=0; y<MAP_HEIGHT; y++){
                  LOG.printfScreen(LOG_LOW, "openings", "%d", cells[x][y]->isBlocked());
                }
                LOG.printfScreen(LOG_LOW, "openings", "\n");
        }
        LOG.printfScreen(LOG_LOW, "openings", "\n");
        LOG.printfScreen(LOG_LOW, "openings", "\n");
        LOG.printfScreen(LOG_LOW, "openings", "\n");
        LOG.printfScreen(LOG_LOW, "openings", "\n");
        for (int x=0; x<MAP_WIDTH; x++){
                for(int y=0; y<MAP_HEIGHT; y++){
                  LOG.printfScreen(LOG_LOW, "openings", "%d", cells[x][y]->getOpenings());
                }
                LOG.printfScreen(LOG_LOW, "openings", "\n");
        }
}

void Map::_adjustOpenings(){
  LOG.printfScreen(LOG_HIGH,"WTF", "YOU GAY\n");
  for (int x=0; x<MAP_WIDTH; x++){
    for(int y=0; y<MAP_HEIGHT; y++){
      if(cells[x][y]->isBlocked())
        continue;
      
      if(x+1<MAP_WIDTH){
        if(!cells[x+1][y]->isBlocked()){
          cells[x][y]->addOpening(RIGHT);
        }
        else {
          cells[x][y]->deleteOpening(RIGHT);
        }
      }
      if(x-1>0){
        if(!cells[x-1][y]->isBlocked()){
          cells[x][y]->addOpening(LEFT);
        }
        else{
          cells[x][y]->deleteOpening(LEFT);
        }
      }
      if(y+1>MAP_HEIGHT){
        if(!cells[x][y+1]->isBlocked()){
          cells[x][y]->addOpening(UP);
        }
        else{
          cells[x][y]->deleteOpening(UP);
        }
      }
      if(y-1>0){
        if(!cells[x][y-1]->isBlocked()){
          cells[x][y]->addOpening(DOWN);
        }
        else{
          cells[x][y]->deleteOpening(DOWN);
        }
      }
    }
  }
}
