/**
 * pose.h
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

#ifndef CS1567_POSE_H
#define CS1567_POSE_H
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

class Pose {
public:
    Pose(float x, float y, float theta);
    ~Pose();
    void reset(float x, float y, float theta);
    void setX(float x);
    void setY(float y);
    void setTheta(float theta);
	float getX();
	float getY();
	float getTheta();
    void add(float deltaX, float deltaY, float deltaTheta);
	void difference(Pose* pose1, Pose* pose2, Pose* destination);
	float distance(Pose* pose1, Pose* pose2);
	void rotateEach(float xAngle, float yAngle, float thetaAngle);
	void rotate(float angle);
	void scale(float sx, float sy);
	void translate(float tx, float ty);
    void toArray(float *arr);
private:
	float _x;
	float _y;
	float _theta;
};

#endif
