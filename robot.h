#ifndef CS1567_ROBOT_H
#define CS1567_ROBOT_H

#include <robot_if++.h>
#include <string>
#include "firfilter.h"

class Robot {
public:
    Robot(std::string address, int id);
    void moveTo(int x, int y);
    void turnTo(int theta);
    void setFailLimit(int limit);
    int getFailLimit();
private:
    RobotInterface *_robotInterface;
    int _failLimit;
    FIRFilter *_nsXFilter;
    FIRFilter *_nsYFilter;
    FIRFilter *_nsThetaFilter;
    FIRFilter *_weLeftFilter;
    FIRFilter *_weRightFilter;
    FIRFilter *_weRearFilter;

    bool _update();
    float _getFilteredLeft();
    float _getFilteredRight();
    float _getFilteredRear();
    float _getFilteredX();
    float _getFilteredY();
    float _getFilteredTheta();
};

#endif
