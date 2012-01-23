#ifndef CS1567_ROBOT_H
#define CS1567_ROBOT_H

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
    //RobotPose getPose();
    //RobotPose getDeltaPose();

    // FIXME: Temporarily public members (for testing)
    //        below
    RobotInterface *_robotInterface;
    bool _update();
    float _getFilteredWELeft();
    float _getFilteredWERight();
    float _getFilteredWERear();
    float _getFilteredNSX();
    float _getFilteredNSY();
    float _getFilteredNSTheta();

    float _getWEDeltaXLeft();
    float _getWEDeltaYLeft();
    float _getWEDeltaXRight();
    float _getWEDeltaYRight();
    float _getWEDeltaXRear();
    float _getWEDeltaYRear();
    float _getWEDeltaX();
    float _getWEDeltaY();
    float _getWEDeltaTheta();
    float _getTransWEDeltaX();
    float _getTransWEDeltaY();
    float _getTransWEDeltaTheta();
    float _getTransNSX();
    float _getTransNSY();
    float _getTransNSTheta();
private:
    // RobotInterface *_robotInterface;

    FIRFilter *_nsXFilter;
    FIRFilter *_nsYFilter;
    FIRFilter *_nsThetaFilter;
    FIRFilter *_weLeftFilter;
    FIRFilter *_weRightFilter;
    FIRFilter *_weRearFilter;

    int _failLimit;

    // bool _update();

    // float _getFilteredWELeft();
    // float _getFilteredWERight();
    // float _getFilteredWERear();
    // float _getFilteredNSX();
    // float _getFilteredNSY();
    // float _getFilteredNSTheta();

    // float _getWEDeltaXLeft();
    // float _getWEDeltaYLeft();
    // float _getWEDeltaXRight();
    // float _getWEDeltaYRight();
    // float _getWEDeltaXRear();
    // float _getWEDeltaYRear();
    // float _getWEDeltaX();
    // float _getWEDeltaY();
    // float _getWEDeltaTheta();
    // float _getTransWEDeltaX();
    // float _getTransWEDeltaY();
    // float _getTransWEDeltaTheta();
    // float _getTransNSDeltaX();
    // float _getTransNSDeltaY();
    // float _getTransNSDeltaTheta();
};

#endif
