#include "map_strategy.h"

MapStrategy::MapStrategy(Map *map) {
	_map = map;
}

MapStrategy::~MapStrategy() {}

Cell* MapStrategy::nextCell() {
	_map->update();

	return NULL;
}