#ifndef CS1567_ROBOT_H
#define CS1567_ROBOT_H

#include "pose.h"
#include "firfilter.h"
#include "kalman.h"
#include "PID.h"
#include "utilities.h"
#include "constants.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <robot_if++.h>
#include <string>

#define ROSIE 0
#define BENDER 1
#define JOHNNY5 2
#define OPTIMUS 3
#define WALLE 4
#define GORT 5

const std::string ROBOTS[] = {
    "rosie",
    "bender",
    "johnny5",
    "optimus",
    "walle",
    "gort"
};

// FIXME: replace with real addresses
const std::string ROBOT_ADDRESSES[] = {
    "192.168.1.41",
    "192.168.1.42",
    "192.168.1.43",
    "192.168.1.44",
    "192.168.1.45",
    "192.168.1.46",
    "192.168.1.47"
};

class Robot {
public:
    Robot(std::string address, int id);
    ~Robot();
    //moves to the given x/y coordinates, without stopping
    void moveTo(float x, float y);
    float moveToUntil(float x, float y, float thetaErrorLimit);
    void turnTo(float theta, float thetaErrorLimit);

	void prefillData();
    void moveForward(int speed);
    void turnLeft(int speed);
    void turnRight(int speed);
    void stop();
    void setFailLimit(int limit);
    int getFailLimit();
    void update();
    Pose* getPose();

    /// Self explanatory
    //TODO: FINDBITCHES
    bool isThereABitchInMyWay();
	int getStrength();

    int getRoom();

    void printBeginDialog();
    void printSuccessDialog();
    void printFailDialog();

    std::string name;

    // FIXME: Temporarily public members (for testing)
    //        below
    RobotInterface *_robotInterface;
    bool _updateInterface();
    float _getWEDeltaLeft();
    float _getWEDeltaRight();
    float _getWEDeltaRear();
    float _getNSX();
    float _getNSY();
    float _getNSTheta();

    float _getWEDeltaXLeft();
    float _getWEDeltaYLeft();
    float _getWEDeltaXRight();
    float _getWEDeltaYRight();
    float _getWEDeltaXRear();
    float _getWEDeltaYRear();
    float _getWEDeltaX();
    float _getWEDeltaY();
    float _getWEDeltaTheta();

    float _getWETransDeltaX();
    float _getWETransDeltaY();
    float _getWETransDeltaTheta();

    float _getNSTransX();
    float _getNSHalfTransX();
    float _getNSTransY();
    float _getNSHalfTransY();
    float _getNSTransTheta();

    void _updateWEPose();
    void _updateNSPose();

    PID* _distancePID;
    PID* _thetaPID;
	
	
    Pose *_wePose;
    Pose *_nsPose;
    Pose *_pose;
	
	
private:
    // RobotInterface *_robotInterface;

    FIRFilter *_nsXFilter;
    FIRFilter *_nsYFilter;
    FIRFilter *_nsThetaFilter;
    FIRFilter *_weLeftFilter;
    FIRFilter *_weRightFilter;
    FIRFilter *_weRearFilter;

	Pose *_startingNSPose;

	bool _passed2PIns;
	bool _passed2PIwe;
	
	// stores the most recent speed that the robot was told. set during both turns and moving straight instructions
	int _speed;
	
	// distance the robot traveled in the time specified below
	float _speedDistance;
	//time values for forward velocity, in terms of time per _speedDistance.  must divide by _speedDistance to get velocity
	float _forwardSpeed[11];
	//time values for turning velocity. need to be divided by a constant to get it into radians
	//these times are time to turn 2*PI radians
	float _turnSpeed[2][11];	
	char _turnDirection;			// value of 1 means turning right, 0 means left
	bool _movingForward;		// set if moving forward/stopped

    int _failLimit;


    Kalman *_kalmanFilter;
    // FIXME: Temporarily not private members (for testing)
    //        below
    // bool _update();

    // float _getWEDeltaLeft();
    // float _getWEDeltaRight();
    // float _getWEDeltaRear();
    // float _getNSX();
    // float _getNSY();
    // float _getNSTheta();

    // float _getWEDeltaXLeft();
    // float _getWEDeltaYLeft();
    // float _getWEDeltaXRight();
    // float _getWEDeltaYRight();
    // float _getWEDeltaXRear();
    // float _getWEDeltaYRear();
    // float _getWEDeltaX();
    // float _getWEDeltaY();
    // float _getWEDeltaTheta();

    // float _getWETransDeltaX();
    // float _getWETransDeltaY();
    // float _getWETransDeltaTheta();

    // float _getNSTransX();
    // float _getNSTransY();
    // float _getNSTransTheta();

    // void _updateWEPose();
    // void _updateNSPose();
};

#endif
