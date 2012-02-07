#include <robot_if++.h>
#include <iostream>
#include <time.h>
#include <sys/time.h>
#include <string>
#include <math.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include "firfilter.h"
#include "utilities.h"
#include "robot.h"
#include "kalmanfilter.h"


int zero_x;
int zero_y;
float zero_theta;
int starting_room_ID;

int main(int argc, char *argv[]) {
	
//Base locations within the global coordinate system
#define NUMBASES 7
	Pose * bases[NUMBASES];
	bases[0] = new Pose(0,0,0);
	bases[1] = new Pose(3.41,0,0);
	bases[2] = new Pose(2.40,1.86,0);
	bases[3] = new Pose(3.20,1.86,0);
	bases[4] = new Pose(4.02,3.03,0);
	bases[5] = new Pose(4.02,3.53,0);
	bases[6] = new Pose(0.69,4.19,0);

	FIRFilter *filtX = new FIRFilter("../filters/ns_x.ffc");
	FIRFilter *filtY = new FIRFilter("../filters/ns_y.ffc");
	FIRFilter *filtTheta = new FIRFilter("../filters/ns_theta.ffc");

	Robot *robot = new Robot(argv[1],0);


	for(int i=0; i<15; i++){
		printf("prefilling data\n");
		robot->update();
	}

	do {
		// Update the robot's sensor information
		
		robot->update();
		
		printf("X global: %d\t\tY global: %d\t\tTheta global: %d\n",robot->_getNSTransX(),robot->_getNSTransY(),robot->_getNSTransTheta());
		

		//robot->Move(RI_MOVE_FORWARD, 1);
		
	} while(1);

	delete(robot);
	return 0;
}
