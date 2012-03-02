#include "pose.h"
#include "constants.h"
#include "utilities.h"

Pose::Pose(float x, float y, float theta) {
	_x = x;
	_y = y;
	_theta = theta;
    // total theta is tracked as well so that it can be used in the
    // kalman filter without returning a weird average for theta
	_totalTheta = theta;
	_numRotations = 0;
}

Pose::~Pose() {}

/* Sets the difference between 2 poses in a given result pose */
void Pose::difference(Pose* returnPose, Pose* pose1, Pose* pose2) {
    returnPose->setX(pose2->getX() - pose1->getX());
    returnPose->setY(pose2->getY() - pose1->getY());
    returnPose->setTotalTheta(pose2->getTotalTheta() - pose1->getTotalTheta());
}

/* Returns the distance (x/y) between 2 poses */
float Pose::distance(Pose* pose1, Pose* pose2) {
	float xerr = pose2->getX()-pose1->getX();
	float yerr = pose2->getY()-pose1->getY();
	return sqrt(xerr*xerr + yerr*yerr);
}

void Pose::setX(float x) {
	_x = x;
}

void Pose::setY(float y) {
	_y = y;
}

/* Sets theta, as well as total theta according to the number of rotations */
void Pose::setTheta(float theta) {
	_theta = fmod(theta, 2*PI);
	_totalTheta = _numRotations * 2*PI + _theta;
}

/* Adds the given number to the pre-existing number of stored rotations */
void Pose::modifyRotations(int num) {
	setNumRotations(_numRotations + num);
}

/* Directly sets total theta, which in turn sets the number of rotations
 * as well as theta
 */
void Pose::setTotalTheta(float totalTheta) {
	_totalTheta = totalTheta;
	_numRotations = (int) _totalTheta/(2*PI);
	_theta = fmod(totalTheta, 2*PI);
	if (_theta < 0) {
		_theta += 2*PI;
	}
}

float Pose::getTotalTheta() {
	return _totalTheta;
}

/* Sets the number of rotations, which track how many times theta has
 * passed over 2PI 
 */
void Pose::setNumRotations(int rot) {
	_numRotations = rot;
	_totalTheta = _numRotations * 2*PI + _theta;
}

int Pose::getNumRotations() {
	return _numRotations;
}

/* Adds the deltas to the current pose, resulting in a new one */
void Pose::add(float deltaX, float deltaY, float deltaTheta) {
	_x += deltaX;
	_y += deltaY;
	_theta += deltaTheta;
}

/* Sets a 3-element array to have x, y, and total theta to be used
 * by a Kalman filter
 */
void Pose::toArrayForKalman(float *arr) {
	arr[0] = _x;
	arr[1] = _y;
	float totalTheta = getTotalTheta();
	arr[2] = totalTheta; // used fabs(totalTheta);
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
