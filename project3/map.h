#ifndef CS1567_MAP_H
#define CS1567_MAP_H

#include <robot_if++.h>
#include "cell.h"

#define MAP_WIDTH 7
#define MAP_HEIGHT 5

#define DIR_NORTH 1
#define DIR_EAST 2
#define DIR_SOUTH 4
#define DIR_WEST 8


class Map {
public:
	Map(RobotInterface *robotInterface, int startingX, int startingY);
	~Map();
	void update();
	int getRobot1Score();
	int getRobot2Score();
	Cell* getCurrentCell();
	bool occupyCell(int x, int y);
	bool reserveCell(int x, int y);

	Cell *cells[MAP_WIDTH][MAP_HEIGHT];
private:
	void _claimRobotAt(int x, int y);
	void _loadMap();
        
    void _adjustOpenings();

	RobotInterface *_robotInterface;

	int _score1;
	int _score2;

	Cell *_curCell;
};

#endif
