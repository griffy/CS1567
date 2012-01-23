#include "robot.h"
#include "utilities.h"

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

Robot::~Robot() {
    delete _robotInterface;
    delete _nsXFilter;
    delete _nsYFilter;
    delete _nsThetaFilter;
    delete _weLeftFilter;
    delete _weRightFilter;
    delete _weRearFilter;
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

float Robot::_getWEDeltaLeft() {
    int left = _robotInterface->getWheelEncoder(RI_WHEEL_LEFT);
    return _weLeftFilter->filter((float) left);
}

float Robot::_getWEDeltaRight() {
    int right = _robotInterface->getWheelEncoder(RI_WHEEL_RIGHT);
    return _weRightFilter->filter((float) right);
}

float Robot::_getWEDeltaRear() {
    int rear = _robotInterface->getWheelEncoder(RI_WHEEL_REAR);
    return _weRearFilter->filter((float) rear);
}

float Robot::_getNSX() {
    int x = _robotInterface->X();
    return _nsXFilter->filter((float) x);
}

float Robot::_getNSY() {
    int y = _robotInterface->Y();
    return _nsYFilter->filter((float) y);
}

float Robot::_getNSTheta() {
    float theta = _robotInterface->Theta();
    return _nsThetaFilter->filter(theta);
}

float Robot::_getWEDeltaXLeft() {
    float deltaX = _getWEDeltaLeft();
    deltaX *= cos(DEGREE_150);
    return deltaX;
}

float Robot::_getWEDeltaYLeft() {
    float deltaY = _getWEDeltaLeft();
    deltaY *= sin(DEGREE_150);
    return deltaY;
}

float Robot::_getWEDeltaXRight() {
    float deltaX = _getWEDeltaRight();
    deltaX *= cos(DEGREE_30);
    return deltaX;
}

float Robot::_getWEDeltaYRight() {
    float deltaY = _getWEDeltaRight();
    deltaY *= sin(DEGREE_30);
    return deltaY;
}

float Robot::_getWEDeltaXRear() {
    return _getWEDeltaRear();
}

float Robot::_getWEDeltaYRear() {
    return 0;
}

float Robot::_getWEDeltaX() {
    float leftDeltaX = _getWEDeltaXLeft();
    float rightDeltaX = _getWEDeltaXRight();
    float rearDeltaX = _getWEDeltaXRear();
    // return the average
    return (leftDeltaX + rightDeltaX + rearDeltaX) / 3;
}

float Robot::_getWEDeltaY() {
    float leftDeltaY = _getWEDeltaYLeft();
    float rightDeltaY = _getWEDeltaYRight();
    // return the average
    return (leftDeltaY + rightDeltaY) / 2;
}

// FIXME: does this really work?
float Robot::_getWEDeltaTheta() {
    float rear = _getWEDeltaRear();
    return rear / (PI * Util::cmToWE(ROBOT_DIAMETER));
}

float Robot::_getTransWEDeltaX() {
    float deltaX = _getWEDeltaX();
    float scaledDeltaX = Util::weToCM(deltaX);
    // TODO: finish
}

float Robot::_getTransWEDeltaY() {
    float deltaY = _getWEDeltaY();
    float scaledDeltaY = Util::weToCM(deltaY);
    // TODO: finish
}

float Robot::_getTransWEDeltaTheta() {
    float deltaTheta = _getWEDeltaTheta();
    // TODO: finish
    return deltaTheta;
}

float Robot::_getTransNSX() {
    
}

float Robot::_getTransNSY() {
    
}

float Robot::_getTransNSTheta() {
    
}