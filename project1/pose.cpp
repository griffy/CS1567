#include "pose.h"

Pose::Pose(float x, float y, float theta) {
	_x = x;
	_y = y;
	_theta = theta;
}

Pose::~Pose() {
	
}

void Pose::setX(float x) {
	_x = x;
}

void Pose::setY(float y) {
	_y = y;
}

void Pose::setTheta(float theta) {
	_theta = theta;
}

void Pose::add(float deltaX, float deltaY, float deltaTheta) {
	_x += deltaX;
	_y += deltaY;
	_theta += deltaTheta;
}

void Pose::toArray(float *arr) {
	arr[0] = _x;
	arr[1] = _y;
	arr[2] = _theta;
}

float Pose::getX(){
	return _x;
}

float Pose::getY(){
	return _y;
}

float Pose::getTheta(){
	return _theta;
}