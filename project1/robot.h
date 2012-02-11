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

class Robot {
public:
    Robot(std::string address, int id);
    ~Robot();
	//moves to the given x/y coordinates, without stopping
    bool moveToFull(int x, int y);
    void moveTo(int x, int y);
	
	//incrementally moves towards given theta
    float turnTo(int theta);
	
    void moveForward(int speed);
    void setFailLimit(int limit);
    int getFailLimit();
    void update();
    Pose* getPose();
	
	/// Self explanatory
	//TODO: FINDBITCHES
	bool isThereABitchInMyWay();
	
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
    float _getNSTransY();
    float _getNSTransTheta();

    void _updateWEPose();
    void _updateNSPose();
	
	PID* _distancePID;
	PID* _thetaPID;
private:
    // RobotInterface *_robotInterface;

    FIRFilter *_nsXFilter;
    FIRFilter *_nsYFilter;
    FIRFilter *_nsThetaFilter;
    FIRFilter *_weLeftFilter;
    FIRFilter *_weRightFilter;
    FIRFilter *_weRearFilter;

    int _failLimit;

    Pose *_wePose;
    Pose *_nsPose;
    Pose *_pose;

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
