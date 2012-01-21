#include <cstdio>
#include <robot_if++.h>
#include "fakerobotinterface.h"
#include "../firfilter.h"

int main() {
    FakeRobotInterface *robot = new FakeRobotInterface("192.168.1.42", 0);
    while (robot->update() == RI_RESP_SUCCESS) {
        printf("%d,%d,%d\n", robot->getWheelEncoder(RI_WHEEL_LEFT),
                             robot->getWheelEncoder(RI_WHEEL_RIGHT),
                             robot->getWheelEncoder(RI_WHEEL_REAR));
    }
    delete robot;
    return 0;
}
