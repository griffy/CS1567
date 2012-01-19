#include "robot.h"

Robot::Robot(std::string address, int id) {
    _robotInterface = new RobotInterface(address, id);
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