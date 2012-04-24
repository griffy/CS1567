/**
 * path.h
 * 
 * @brief 
 * 		This class represents a path the robot could travel in the
 *      game, and all the operations necessary to work with it, including
 *      calculating its value.
 * 
 * @author
 * 		Shawn Hanna
 * 		Tom Nason
 * 		Joel Griffith
 *
 **/

#ifndef CS1567_PATH_H
#define CS1567_PATH_H

#include "cell.h"
#include "logger.h"
#include <vector>

class Path {
public:
	Path();
	Path(Cell *cell);
	Path(Path *path);
	~Path();
	void push(Cell *cell);
	void pop();
	int length();
	int getHeading(int upTo);
	float getValue();
	Cell* getCell(int i);
	Cell* getFirstCell();
	Cell* getLastCell();
	char* toString();
private:
	std::vector<Cell*> _cells;
};

#endif