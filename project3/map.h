#ifndef CS1567_MAP_H
#define CS1567_MAP_H

#include <robot_if++.h>
#include "cell.h"

#define MAP_WIDTH 7
#define MAP_HEIGHT 5

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
	void _setRobotAt(int x, int y);
	void _loadMap();
        
        void _adjustOpenings();

	RobotInterface *_robotInterface;

	int _score1;
	int _score2;

	Cell *_curCell;
};

#endif
