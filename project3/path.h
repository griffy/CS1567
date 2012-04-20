#ifndef CS1567_PATH_H
#define CS1567_PATH_H

#include "map.h"

NS_PENALTY[MAP_WIDTH][MAP_HEIGHT] = {
	// col 1 (starting from left of room)
	{0.60,
	 0.55,
	 0.50,
	 0.50,
	 0.50},
	// col 2
	{0.35,
	 0.30,
	 0.25,
	 0.25,
	 0.25},
	// col 3
	{0.10,
	 0.05,
	 0.00,
	 0.00,
	 0.00},
	// col 4
	{0.10,
	 0.05,
	 0.00,
	 0.00,
	 0.00},
	// col 5
	{0.10,
	 0.05,
	 0.00,
	 0.00,
	 0.00},
	// col 6
	{0.15,
	 0.10,
	 0.00,
	 0.00,
	 0.00},
	// col 7
	{0.20,
	 0.15,
	 0.01,
	 0.00,
	 0.00}
};

class Path {
public:
	Path();
	Path(Cell *cell);
	Path(Path *path);
	~Path();
	double getValue();
	Cell* getCell(int i);
	Cell* getFirstCell();
	Cell* getLastCell();
	void push(Cell *);
	Cell* pop();
private:
	std::vector<Cell*> _cells;
};

#endif