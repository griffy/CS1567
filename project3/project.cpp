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
	if (argc < 3){
		printf("need argument for rovio man id (1 or 2)\n");
		return -1;
	}

    LOG.setImportanceLevel(LOG_LOW);
	
	Robot *robot = new Robot(argv[1], atoi(argv[2]));

//	robot->move(DIR_EAST, 2); // cells
//	robot->move(DIR_SOUTH, 2);
//	robot->move(DIR_WEST, 2);
//	robot->move(DIR_NORTH, 2);

	robot->eatShit();

	delete robot;

	return 0;
}
