#include "pose.h"

Pose::Pose(float x, float y, float theta) {
	_x = x;
	_y = y;
	_theta = theta;
}

Pose::~Pose() {
	
}

Pose* Pose::plus(float deltaX, float deltaY, float deltaTheta) {
	float newX = _x + deltaX;
	float newY = _y + deltaY;
	float newTheta = _theta + deltaTheta;
	return new Pose(newX, newY, newTheta);
}

void Pose::toArray(float *arr) {
	arr[0] = _x;
	arr[1] = _y;
	arr[2] = _theta;
}