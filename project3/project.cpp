/**
 * project.cpp
 * 
 * @brief 
 * 		This program commands the robot to perform the task of 
 *      moving down the corridor. It has functionality that enables
 * 		it to call functions and access members in the Robot class
 * 
 * @author
 * 		Shawn Hanna
 * 		Tom Nason
 * 		Joel Griffith
 * 
 **/

#include "robot.h"
#include "logger.h"
#include "camera.h"
#include <stdio.h>

int main(int argc, char *argv[]) {
	if (argc < 2) {
		printf("ERROR: need argument for robot name\n");
		return -1;
	}

    LOG.setImportanceLevel(LOG_HIGH);

	Robot *robot = new Robot(argv[1], 0);

	/*while (true) {
		bool turn = false;
		robot->updateCamera();
		robot->_camera->centerError(COLOR_PINK, &turn);
	}*/
	
	robot->move(DIR_EAST, 4); // cells
	robot->move(DIR_SOUTH, 3);

	delete robot;

	return 0;
}
