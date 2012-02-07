#ifndef CS1567_POSE_H
#define CS1567_POSE_H

class Pose {
public:
    Pose(float x, float y, float theta);
    ~Pose();
    void setX(float x);
    void setY(float y);
    void setTheta(float theta);
    void add(float deltaX, float deltaY, float deltaTheta);
    void toArray(float *arr);

	float getX();
	float getY();
	float getTheta();
private:
	float _x;
	float _y;
	float _theta;
};

#endif