class Map {
public:
	Map(RobotInterface *robotInterface);
	~Map();
	void update();
	int getTeam1Score();
	int getTeam2Score();
	
private:
	void _loadMap();

	RobotInterface *_robotInterface;

	int _score1;
	int _score2;

	Cell *_cells[7][5];
};