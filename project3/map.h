#include <robot_if++.h>
#include "cell.h"

class Map {
public:
	Map(RobotInterface *robotInterface);
	~Map();
	void update();
	int getTeam1Score();
	int getTeam2Score();
	bool occupyCell(int x, int y);
	bool reserveCell(int x, int y);
	void setOpenings(int x, int y);

	Cell *cells[7][5];
private:
	void _loadMap();

	RobotInterface *_robotInterface;

	int _score1;
	int _score2;
};