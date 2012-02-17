#include "pose.h"
#include "constants.h"


Pose::Pose(float x, float y, float theta) {
	_x = x;
	_y = y;
	_theta = theta;
	_totalTheta = theta;
	_numRotations=0;
}

Pose::~Pose() {
	
}

//gets the difference between 2 poses
// make sure to delete after use
void Pose::difference(Pose* returnPose, Pose* pose1, Pose* pose2){
	delete returnPose;
	returnPose = new Pose(pose2->getX()-pose1->getX(), pose2->getY()-pose1->getY(),pose2->getTotalTheta()-pose1->getTotalTheta());
}

//gets the distance (x/y) between 2 poses
// make sure to delete after use
float Pose::distance(Pose* pose1, Pose* pose2){
	float xerr=pose2->getX()-pose1->getX();
	float yerr=pose2->getY()-pose1->getY();
	return sqrt(xerr*xerr+yerr*yerr);
}

void Pose::setX(float x) {
	_x = x;
}

void Pose::setY(float y) {
	_y = y;
}

void Pose::setTheta(float theta) {
	_theta = fmod(theta,2*PI);
}

void Pose::modifyRotations(int num) {
	_numRotations+=num;
}

float Pose::getTotalTheta(){
	_totalTheta=_numRotations*2*PI+_theta;
	return _totalTheta;
}

float Pose::normalizeTotalTheta(){
	float temp = fmod(getTotalTheta(), 2*PI);
	if(temp < 0){
		temp+=2*PI;
	}
	return temp;
}


void Pose::setTotalTheta(float t){
	_totalTheta=t;
	_numRotations=(int) _totalTheta/(2*PI);
	_theta=fmod(t,2*PI);
	if(_theta < 0){
		_theta+=2*PI;
	}
}

void Pose::setNumRotations(int rot) {
	_numRotations = rot;
}

int Pose::getNumRotations(){
	return _numRotations;
}

void Pose::add(float deltaX, float deltaY, float deltaTheta) {
	_x += deltaX;
	_y += deltaY;
	_theta += deltaTheta;
}

void Pose::toArray(float *arr) {
	arr[0] = _x;
	arr[1] = _y;
	arr[2] = getTotalTheta();
}

float Pose::getX() {
	return _x;
}

float Pose::getY(){
	return _y;
}

float Pose::getTheta(){
	return _theta;
}
