#ifndef CS1567_MAPSTRATEGY_H
#define CS1567_MAPSTRATEGY_H

#include "map.h"
#include "cell.h"

#define MAX_SEARCH_DEPTH 7

#define CELL_NORTH 0
#define CELL_SOUTH 1
#define CELL_EAST 2
#define CELL_WEST 3

typedef struct move {
	int direction;
	struct move *next;
	int cost;
} Move;

class MapStrategy {
public:
	MapStrategy(Map *map);
	~MapStrategy();
	Cell* nextCell();

private:
	Map *_map;
};

#endif
