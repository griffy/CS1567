#include "robot.h"
#include<stdio.h>
#include <string>

int main() {
    Robot *robot = new Robot("192.168.1.44", 0);
    robot->update();
    float y = 0;
    while (y < 300 * 4) {
    	robot->_robotInterface->Move(RI_MOVE_FORWARD, 1);
    	robot->update();
    	float deltaY = robot->_getWEDeltaY();
    	y += deltaY;
    	printf("%f\n", y);
    }
    /*
    for (;;) {
    	robot->_update();
    	printf("(%f, %f), (%f, %f), (%f, %f) -> %f, %f, %f\n", robot->_getWEDeltaXLeft() / 4,
    								   robot->_getWEDeltaYLeft() / 4,
    								   robot->_getWEDeltaXRight() / 4,
    								   robot->_getWEDeltaYRight() / 4,
    								   robot->_getWEDeltaXRear() / 4,
    								   robot->_getWEDeltaYRear() / 4,
    								   robot->_getWEDeltaX() / 4,
    								   robot->_getWEDeltaY() / 4,
    								   robot->_getWEDeltaTheta());
  		robot->_robotInterface->Move(RI_TURN_RIGHT, 1);
    }
    */
    delete(robot);
    return 0;
}
