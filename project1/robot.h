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

    int nameToInt();
    void printOpeningDialog();
    void printSuccessDialog();
    void printFailureDialog();

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
