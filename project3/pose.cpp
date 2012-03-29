/**
 * pose.cpp
 * 
 * @brief 
 * 		This class is used for keeping the robot's physical pose (x, y, theta) 
 *      and performing calculations on it.
 * 
 * @author
 * 		Shawn Hanna
 * 		Tom Nason
 * 		Joel Griffith
 *
 **/

#include "pose.h"
#include "constants.h"
#include "utilities.h"

Pose::Pose(float x, float y, float theta) {
	_x = x;
	_y = y;
	_theta = theta;
}

Pose::~Pose() {}

/**************************************
 * Definition: Stores the difference between two poses in a third pose
 *
 * Parameters: pointers to two poses, along with a pointer to a result pose
 **************************************/
void Pose::difference(Pose* pose1, Pose* pose2, Pose* returnPose) {
    returnPose->setX(pose2->getX() - pose1->getX());
    returnPose->setY(pose2->getY() - pose1->getY());
    returnPose->setTheta(pose2->getTheta() - pose1->getTheta());
}

/**************************************
 * Definition: Returns the distance between two poses
 *
 * Parameters: pointers to two poses
 *
 * Returns:    distance as a float
 **************************************/
float Pose::distance(Pose* pose1, Pose* pose2) {
	float xerr = pose2->getX() - pose1->getX();
	float yerr = pose2->getY() - pose1->getY();
	return sqrt(xerr*xerr + yerr*yerr);
}

/**************************************
 * Definition: Resets the current pose according to parameters
 *
 * Parameters: a new x, y, and theta
 **************************************/
void Pose::reset(float x, float y, float theta) {
	setX(x);
	setY(y);
	setTheta(theta);
}

/**************************************
 * Definition: Sets the current pose's x to the given x
 *
 * Parameters: x as a float
 **************************************/
void Pose::setX(float x) {
	_x = x;
}

/**************************************
 * Definition: Sets the current pose's y to the given y
 *
 * Parameters: y as a float
 **************************************/
void Pose::setY(float y) {
	_y = y;
}

/**************************************
 * Definition: Sets the current pose's theta to the given theta
 *
 * Parameters: theta as a float
 **************************************/
void Pose::setTheta(float theta) {
	_theta = Util::normalizeTheta(theta);
}

/**************************************
 * Definition: Adds deltas to the current pose
 *
 * Parameters: delta x, delta y, and delta theta as floats
 **************************************/
void Pose::add(float deltaX, float deltaY, float deltaTheta) {
	_x += deltaX;
	_y += deltaY;
	_theta += deltaTheta;
}

/**************************************
 * Definition: Sets a 3-element array to have x, y, and theta 
 *
 * Parameters: pointer to an array
 **************************************/
void Pose::toArray(float *arr) {
	arr[0] = _x;
	arr[1] = _y;
	arr[2] = _theta;
}

/**************************************
 * Definition: Returns x
 *
 * Returns:    x as an int
 **************************************/
float Pose::getX() {
	return _x;
}

/**************************************
 * Definition: Returns y
 *
 * Returns:    y as an int
 **************************************/
float Pose::getY(){
	return _y;
}

/**************************************
 * Definition: Returns theta
 *
 * Returns:    theta as an int
 **************************************/
float Pose::getTheta(){
	return _theta;
}

/**************************************
 * Definition: Rotates x, y, and theta according to the
 *             given angles
 *
 * Parameters: x angle, y angle, and theta angle as floats
 **************************************/
void Pose::rotateEach(float xAngle, float yAngle, float thetaAngle) {
	float newX = getX() * cos(xAngle) - getY() * sin(xAngle);
	float newY = getX() * sin(yAngle) + getY() * cos(yAngle);
	float newTheta = getTheta() - thetaAngle;
    setX(newX);
    setY(newY);
    setTheta(newTheta);
}

/**************************************
 * Definition: Rotates x, y, and theta according to the
 *             given angle
 *
 * Parameters: angle as a float
 **************************************/
void Pose::rotate(float angle) {
	float newX = getX() * cos(angle) - getY() * sin(angle);
	float newY = getX() * sin(angle) + getY() * cos(angle);
	float newTheta = getTheta() - angle;
    setX(newX);
    setY(newY);
    setTheta(newTheta);
}

/**************************************
 * Definition: Scales x and y according to the scaling constants
 *
 * Parameters: x and y scaling floats
 **************************************/
void Pose::scale(float sx, float sy) {
	float newX = getX() / sx;
	float newY = getY() / sy;
    setX(newX);
    setY(newY);
}

/**************************************
 * Definition: Translates x and y according to translation constants
 *
 * Parameters: x and y translation floats
 **************************************/
void Pose::translate(float tx, float ty) {
	float newX = getX() + tx;
	float newY = getY() + ty;
    setX(newX);
    setY(newY);
}
