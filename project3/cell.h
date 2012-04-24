/**
 * cell.h
 * 
 * @brief 
 * 		This class is used to represent a single cell in a grid, and
 *      all the operations on that cell (including updates from the
 *      game server)
 * 
 * @author
 * 		Shawn Hanna
 * 		Tom Nason
 * 		Joel Griffith
 * 
 **/

#ifndef CS1567_CELL_H
#define CS1567_CELL_H

#include <robot_if++.h>

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
	
	static int robot;
	
	int x;
	int y;
private:
	int _type;

	int _points;
	bool _blocked;
	bool _post;
	bool _occupied;
	bool _reserved;
};

#endif