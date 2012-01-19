#ifndef CS1567_ROBOT_H
#define CS1567_ROBOT_H

#include <robot_if++.h>
#include <string>

class Robot {
public:
	Robot(std::string address, int id);
	void moveTo(int x, int y);
	void turnTo(int theta);
	void setFailLimit(int limit);
	int getFailLimit();
private:
	RobotInterface *_robotInterface;
	int _failLimit;

	bool _update();
};

#endif