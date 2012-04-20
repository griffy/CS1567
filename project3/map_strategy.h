#ifndef CS1567_MAPSTRATEGY_H
#define CS1567_MAPSTRATEGY_H

#include "map.h"
#include "cell.h"
#include "path.h"

#define PATH_LENGTH 5

#define CELL_NORTH 0
#define CELL_SOUTH 1
#define CELL_EAST 2
#define CELL_WEST 3

class MapStrategy {
public:
	MapStrategy(Map *map);
	~MapStrategy();
	Cell* nextCell();
	Path* getBestPath(int length);
	void createPaths(Path *parentPath, int length);
	void clearPaths();
	
private:
	Map *_map;
	
	std::vector<Path *> _pathList;
	
	void createPath(Path parentPath, int num);
};

#endif
