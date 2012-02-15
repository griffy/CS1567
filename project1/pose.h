#ifndef CS1567_POSE_H
#define CS1567_POSE_H
#include <math.h>

class Pose {
public:
    Pose(float x, float y, float theta);
    ~Pose();
    void setX(float x);
    void setY(float y);
    void setTheta(float theta);
    void add(float deltaX, float deltaY, float deltaTheta);
    void toArray(float *arr);

	//gets the pose difference between the 2 poses
	void difference(Pose* destination, Pose* pose1, Pose* pose2);

	//gets the distance (x/y) between the 2 poses
	float distance(Pose* pose1, Pose* pose2);
	
	float normalizeTotalTheta();
	float getX();
	float getY();
	float getTheta();
	float getTotalTheta();
	void setNumRotations(int rot);
	int getNumRotations();
	void setTotalTheta(float theta);

	void modifyRotations(int num);
	
	int _numRotations;
	
private:
	float _x;
	float _y;
	float _theta;
	float _totalTheta;
};

#endif
