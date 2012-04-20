#ifndef CS1567_PATH_H
#define CS1567_PATH_H

#include "map.h"
#include "cell.h"
#include <vector>

float NS_PENALTY[MAP_WIDTH][MAP_HEIGHT] = {
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
	 0.01,
	 0.00,
	 0.00},
	// col 4
	{0.10,
	 0.05,
	 0.01,
	 0.00,
	 0.00},
	// col 5
	{0.10,
	 0.05,
	 0.01,
	 0.00,
	 0.00},
	// col 6
	{0.15,
	 0.10,
	 0.01,
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
	void push(Cell *cell);
	void pop();
	int length();
	int getHeading(int upTo);
	float getValue();
	Cell* getCell(int i);
	Cell* getFirstCell();
	Cell* getLastCell();
	void push(Cell *);
	Cell* pop();
private:
	std::vector<Cell*> _cells;
};

#endif