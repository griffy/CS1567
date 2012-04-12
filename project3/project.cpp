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
		robot->turnRight(1);
		robot->turnLeft(1);
		usleep(1000000);
		
		robot->turnRight(2);
		robot->turnLeft(2);
		usleep(1000000);
		
		robot->turnRight(3);
		robot->turnLeft(3);
		usleep(1000000);
		
		robot->turnRight(4);
		robot->turnLeft(4);
		usleep(1000000);

		robot->turnRight(5);
		robot->turnLeft(5);
		usleep(1000000);
		
		robot->turnRight(6);
		robot->turnLeft(6);
		usleep(1000000);
		
		robot->turnRight(7);
		robot->turnLeft(7);
		usleep(1000000);
		
		robot->turnRight(8);
		robot->turnLeft(8);
		usleep(1000000);
		
		robot->turnRight(9);
		robot->turnLeft(9);
		usleep(1000000);

		robot->turnRight(10);
		robot->turnLeft(10);
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
