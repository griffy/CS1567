#include "map_strategy.h"

MapStrategy::MapStrategy(Map *map) {
	_map = map;
}

MapStrategy::~MapStrategy() {}

Cell* MapStrategy::nextCell() {
	_map->update();

	// do the reserving and occupying here
	// before returning the cell
	
	return NULL;
}