#include "robot.h"

#define DEFAULT_NUM_FAILS 5

Robot::Robot(std::string address, int id) {
    _robotInterface = new RobotInterface(address, id);

    _nsXFilter = new FIRFilter(NS_X);
    _nsYFilter = new FIRFilter(NS_Y);
    _nsThetaFilter = new FIRFilter(NS_THETA);
    _weLeftFilter = new FIRFilter(WE);
    _weRightFilter = new FIRFilter(WE);
    _weRearFilter = new FIRFilter(WE);

    setFailLimit(DEFAULT_NUM_FAILS);
}

void Robot::moveTo(int x, int y) {}

void Robot::turnTo(int theta) {}

void Robot::setFailLimit(int limit) {
    _failLimit = limit;
}

int Robot::getFailLimit() {
    return _failLimit;
}

bool Robot::_update() {
    int failCount = 0;
    int failLimit = getFailLimit();

    while (_robotInterface->update() != RI_RESP_SUCCESS &&
           failCount < failLimit) {
        failCount++;
    }

    if (failCount >= failLimit) {
        return false;
    }
    return true;
}

float Robot::_getFilteredLeft() {
    int left = _robotInterface->getWheelEncoder(RI_WHEEL_LEFT);
    return _weLeftFilter->filter((float) left);
}

float Robot::_getFilteredRight() {
    int right = _robotInterface->getWheelEncoder(RI_WHEEL_RIGHT);
    return _weRightFilter->filter((float) right);
}

float Robot::_getFilteredRear() {
    int rear = _robotInterface->getWheelEncoder(RI_WHEEL_REAR);
    return _weRearFilter->filter((float) rear);
}

float Robot::_getFilteredX() {
    int x = _robotInterface->X();
    return _nsXFilter->filter((float) x);
}

float Robot::_getFilteredY() {
    int y = _robotInterface->Y();
    return _nsYFilter->filter((float) y);
}

float Robot::_getFilteredTheta() {
    float theta = _robotInterface->Theta();
    return _nsThetaFilter->filter(theta);
}
