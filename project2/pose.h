#ifndef CS1567_POSE_H
#define CS1567_POSE_H
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

class Pose {
public:
    Pose(float x, float y, float theta);
    ~Pose();
    void setX(float x);
    void setY(float y);
    void setTheta(float theta);
	void setTotalTheta(float totalTheta);
    void add(float deltaX, float deltaY, float deltaTheta);
    void toArrayForKalman(float *arr);

	void difference(Pose* destination, Pose* pose1, Pose* pose2);
	float distance(Pose* pose1, Pose* pose2);
	
	void setNumRotations(int rot);
	void modifyRotations(int num);
	
	float getX();
	float getY();
	float getTheta();
	float getTotalTheta();
	int getNumRotations();
	
	int _numRotations;
	
private:
	float _x;
	float _y;
	float _theta;
	float _totalTheta;
};

#endif
