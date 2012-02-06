#ifndef CS1567_ROBOT_H
#define CS1567_ROBOT_H

#include "pose.h"
#include "firfilter.h"

#include <robot_if++.h>
#include <string>

class Robot {
public:
    Robot(std::string address, int id);
    ~Robot();
    void moveTo(int x, int y);
    void turnTo(int theta);
    void setFailLimit(int limit);
    int getFailLimit();
    Pose* getPose();

    // FIXME: Temporarily public members (for testing)
    //        below
    RobotInterface *_robotInterface;
    bool _update();
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
    
    float _getWETransX();
    float _getWETransY();
    float _getWETransTheta();
    float _getNSTransX();
    float _getNSTransY();
    float _getNSTransTheta();

    Pose* _getWEPose();
    Pose* _getNSPose();
private:
    // RobotInterface *_robotInterface;

    FIRFilter *_nsXFilter;
    FIRFilter *_nsYFilter;
    FIRFilter *_nsThetaFilter;
    FIRFilter *_weLeftFilter;
    FIRFilter *_weRightFilter;
    FIRFilter *_weRearFilter;

    int _failLimit;

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
    
    // float _getWETransX();
    // float _getWETransY();
    // float _getWETransTheta();
    // float _getNSTransX();
    // float _getNSTransY();
    // float _getNSTransTheta();

    // Pose* _getWEPose();
    // Pose* _getNSPose();
};

#endif
