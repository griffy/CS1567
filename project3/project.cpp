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

    LOG.setImportanceLevel(LOG_LOW);

	Robot *robot = new Robot(argv[1], 0);

	while (true) {
		//robot->updateCamera();
		//robot->center();
		robot->strafeRight(1);
		robot->strafeLeft(1);
		usleep(1000000);
		
		robot->strafeRight(2);
		robot->strafeLeft(2);
		usleep(1000000);
		
		robot->strafeRight(3);
		robot->strafeLeft(3);
		usleep(1000000);
		
		robot->strafeRight(4);
		robot->strafeLeft(4);
		usleep(1000000);

		robot->strafeRight(5);
		robot->strafeLeft(5);
		usleep(1000000);
		
		robot->strafeRight(6);
		robot->strafeLeft(6);
		usleep(1000000);
		
		robot->strafeRight(7);
		robot->strafeLeft(7);
		usleep(1000000);
		
		robot->strafeRight(8);
		robot->strafeLeft(8);
		usleep(1000000);
		
		robot->strafeRight(9);
		robot->strafeLeft(9);
		usleep(1000000);

		robot->strafeRight(10);
		robot->strafeLeft(10);
		usleep(1000000);
		//robot->getInterface()->Move(RI_TURN_RIGHT, 1);
		//robot->getInterface()->Move(RI_TURN_RIGHT, 1);
		//robot->getInterface()->Move(RI_TURN_LEFT, 1);
		//usleep(1000000);
	}
	
	//robot->move(DIR_EAST, 4); // cells
	//robot->move(DIR_SOUTH, 3);

	delete robot;

	return 0;
}
