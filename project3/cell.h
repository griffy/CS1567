#ifndef CS1567_CELL_H
#define CS1567_CELL_H

#include <robot_if++.h>

#define CELL_T 0
#define CELL_CORNER 1
#define CELL_HALL 2
#define CELL_INTERSECTION 3

#define RIGHT 1
#define DOWN 2
#define LEFT 4
#define UP 8

class Cell {
public:
	Cell(map_obj_t *mapObj);
	~Cell();
	void claimRobot();
	void update(map_obj_t *mapObj);
	bool occupy(RobotInterface *robotInterface);
	bool reserve(RobotInterface *robotInterface);
	int getPoints();
	void setPoints(int points);
	bool isBlocked();
	bool isPost();
	void setPost(bool post);
	bool isOccupied();
	void setOccupied(bool occupied);
	bool isReserved();
	void setReserved(bool reserved);
	
	void addOpening(unsigned char dir);
	void deleteOpening(unsigned char direction);
    int getOpenings();
	
	int x;
	int y;
private:
	int _type;

	int _points;
	bool _blocked;
	bool _post;
	bool _occupied;
	bool _reserved;

	char _openings;

	static int _robot;
};

#endif