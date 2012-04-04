#include <robot_if++.h>

enum CellType { CORNER, HALL, T, INTERSECTION };

class Cell {
public:
	Cell(map_obj_t *mapObj);
	~Cell();
	void update(map_obj_t *mapObj);
	bool occupy(RobotInterface *robotInterface);
	bool reserve(RobotInterface *robotInterface);
	int getPoints();
	void setPoints(int points);
	bool isBlocked();
	void setBlocked(bool blocked);
	bool isPost();
	void setPost(bool post);
	bool isOccupied();
	void setOccupied(bool occupied);
	bool isReserved();
	void setReserved(bool reserved);
	
	void addOpening(char dir);

	int x;
	int y;
private:
	void _updateBlocked();

	int _points;
	bool _blocked;
	bool _post;
	bool _occupied;
	bool _reserved;
	
	char _openings;
};