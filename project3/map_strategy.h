/**
 * map_strategy.h
 * 
 * @brief 
 *      This class is used to implement path-finding for an instance
 *      of the Map class. 
 *
 * @author
 * 		Shawn Hanna
 * 		Tom Nason
 * 		Joel Griffith
 *
 **/

#ifndef CS1567_MAPSTRATEGY_H
#define CS1567_MAPSTRATEGY_H

#include "map.h"
#include "cell.h"
#include "path.h"

// maximum search depth for path-finding
#define PATH_LENGTH 5

class MapStrategy {
public:
	MapStrategy(Map *map);
	~MapStrategy();
	Cell* nextCell();
	
private:
	Map *_map;
	std::vector<Path *> _pathList;
	
	Path* _getBestPath(int length);
	void _createPaths(Path *parentPath, int length);
	void _clearPaths();
};

#endif
