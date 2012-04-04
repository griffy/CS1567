#ifndef CS1567_MAPSTRATEGY_H
#define CS1567_MAPSTRATEGY_H

#include "map.h"
#include "cell.h"

class MapStrategy {
public:
	MapStrategy(Map *map);
	~MapStrategy();
	Cell* nextCell();

private:
	Map *_map;
};

#endif
