#ifndef CS1567_POSE_H
#define CS1567_POSE_H

class Pose {
public:
    Pose(float x, float y, float theta);
    ~Pose();
    Pose* plus(float deltaX, float deltaY, float deltaTheta);
    void toArray(float *arr);
private:
	float _x;
	float _y;
	float _theta;
};

#endif