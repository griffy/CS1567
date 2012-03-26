/**
 * project.cpp
 * 
 * @brief 
 * 		This program commands the robot to perform a task. It has functionality that enables
 * 		it to call functions and access members in the Robot class
 * 
 * @author
 * 		Joel Griffith
 * 		Shawn Hanna
 * 		Tom Nason
 * 
 * @date
 * 		created - 2/2/2012
 * 		modified - 3/24/2012
 **/

#include "robot.h"
#include "logger.h"
#include "camera.h"
#include <stdio.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <robot_color.h>

#define NUM_BASES 7

int main(int argc, char *argv[]) {
	if (argc < 2) {
		printf("ERROR: need argument for robot name\n");
		return -1;
	}

    LOG.setImportanceLevel(LOG_LOW);

	Robot *robot = new Robot(argv[1], 0);

//	robot->center();
/* Camera Testing Code
	while (true) {
		robot->updateCamera();
		robot->_camera->centerError(COLOR_PINK);
		cvWaitKey(0);
	}
*/

// Project 2 Code
/*	robot->move(DIR_EAST, 5);
	robot->move(DIR_EAST, 5);
	robot->turn(DIR_RIGHT, DEGREE_90);
	robot->move(DIR_SOUTH, 3);
	
// Sensor Testing Code
/*
	while(1) {
		robot->updatePose();
		robot->moveForward(4);
		robot->moveForward(4);
		robot->moveForward(4);
		robot->moveForward(4);
		robot->updatePose();
		robot->turnLeft(6);
		robot->updatePose();
		robot->moveForward(4);
		robot->moveForward(4);
		robot->moveForward(4);
		robot->moveForward(4);
		robot->updatePose();
		robot->turnRight(6);
		robot->updatePose();
		robot->moveForward(4);
		robot->moveForward(4);
		robot->moveForward(4);
		robot->moveForward(4);	
	}
*/

// Project 1 Code
/*
    // Base locations in cm within the global coordinate system
	Pose * bases[NUM_BASES];
	bases[0] = new Pose(300, 0, 0); // base 1
	bases[1] = new Pose(229, 183, 0); // base 2
    bases[2] = new Pose(326, 183, 0); // fake base
	bases[3] = new Pose(392, 300, 0); // base 3
    bases[4] = new Pose(318, 386, 0); // fake base
    bases[5] = new Pose(49, 386, 0); // base 4
    bases[6] = new Pose(0, 0, 0); // base 5/0

    for (int i = 0; i < NUM_BASES; i++) {
    	printf("moving to base %d...\n", i+1);
    	robot->moveTo(bases[i]->getX(), bases[i]->getY());
    	printf("reached base %d!\n", i+1);
    }

    printf("done!\n");

	delete robot;
	for (int i = 0; i < NUM_BASES; i++) {
		delete bases[i];
	}
*/

	delete robot;

	return 0;
}
