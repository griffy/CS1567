#ifndef CS1567_ROBOT_H
#define CS1567_ROBOT_H

#include "pose.h"
#include "camera.h"
#include "wheel_encoders.h"
#include "north_star.h"
#include "fir_filter.h"
#include "kalman_filter.h"
#include "PID.h"
#include "utilities.h"
#include "constants.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <robot_if++.h>
#include <string>

#define NUM_SPEEDS 11

const float TIME_DISTANCE = 116.0; // cm

// average speed to move forward TIME_DISTANCE at integer robot speeds
const float SPEED_FORWARD[NUM_SPEEDS] = {
    0.0,
    TIME_DISTANCE/3.5,
    TIME_DISTANCE/3.5,
    TIME_DISTANCE/3.5,
    TIME_DISTANCE/3.7,
    TIME_DISTANCE/3.7,
    TIME_DISTANCE/3.9,
    TIME_DISTANCE/4.6,
    TIME_DISTANCE/4.6,
    TIME_DISTANCE/4.8,
    TIME_DISTANCE/4.9
};

// average speed for left and right turns at integer robot speeds
const float SPEED_TURN[][NUM_SPEEDS] = {
    {0.0, 0.0},
    {(2*PI)/1.8, -(2*PI)/1.6},
    {(2*PI)/1.9, -(2*PI)/2.0},
    {(2*PI)/2.23, -(2*PI)/2.2},
    {(2*PI)/2.36, -(2*PI)/2.3},
    {(2*PI)/2.87, -(2*PI)/2.8},
    {(2*PI)/2.8, -(2*PI)/2.8},
    {(2*PI)/4.6, -(2*PI)/4.75},
    {(2*PI)/4.75, -(2*PI)/4.75},
    {(2*PI)/5.35, -(2*PI)/5.35},
    {(2*PI)/5.35, -(2*PI)/5.45}
};

class Robot {
public:
    Robot(std::string address, int id);
    ~Robot();
    //moves to the given x/y coordinates, without stopping
    void moveTo(float x, float y);
    float moveToUntil(float x, float y, float thetaErrorLimit);
    void turnTo(float theta, float thetaErrorLimit);
    void centerSelf(int maxError);

    void setFailLimit(int limit);
    int getFailLimit();
    void setCameraResolution(int resolution, int quality);
	void prefillData();
    void update();

    void moveForward(int speed);
    void turnLeft(int speed);
    void turnRight(int speed);
    void stop();

    Pose* getPose();

    // Self explanatory
    bool isThereABitchInMyWay();
	int getStrength();
    int getRoom();
    int getBattery();

    void printBeginPhrase();
    void printSuccessPhrase();
    void printFailPhrase();
    void rockOut();

    Camera *_camera;
private:
    RobotInterface *_robotInterface;
    int _name;

	// stores the most recent speed that the robot was told. set during both turns and moving straight instructions
	int _speed;	
	char _turnDirection;			// value of 1 means turning right, 0 means left
	bool _movingForward;		// set if moving forward/stopped

    PID* _distancePID;
    PID* _thetaPID;

    int _failLimit;

    Pose *_pose;

    //Camera *_camera;
    WheelEncoders *_wheelEncoders;
    NorthStar *_northStar;

    KalmanFilter *_kalmanFilter;

    bool _updateInterface();
};

#endif
