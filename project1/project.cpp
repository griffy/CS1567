#include "robot.h"
#include <iostream>
#include <time.h>
#include <sys/time.h>
#include <string>
#include <math.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>

#define NUMBASES 7

int main(int argc, char *argv[]) {
	if (argc < 2) {
		printf("ERROR: need argument for robot name\n");
		return -1;
	}

    //Base locations within the global coordinate system
	Pose * bases[NUMBASES];
	bases[0] = new Pose(341, 0, 0);
	bases[1] = new Pose(240, 186, 0);
	bases[2] = new Pose(320, 186, 0);
	bases[3] = new Pose(402, 303, 0);
	bases[4] = new Pose(402, 353, 0);
	bases[5] = new Pose(69, 419, 0);
	bases[6] = new Pose(0, 0, 0);

	Robot *robot = new Robot(argv[1], 0);
	printf("Battery level: %d\n", robot->_robotInterface->Battery());
	
	printf("battery: %d\n", robot->_robotInterface->Battery());

    for (int i = 0; i < 1; i++) {
    	printf("moving to base %d...\n", i+1);
    	robot->moveTo(bases[i]->getX(), bases[i]->getY());
    	printf("reached base %d!\n", i+1);
		
		printf("prefilling data\n");
		for (int j = 0; j < 5; j++) {
			robot->update();
		}
    }

    printf("done!");

	delete robot;
	for (int i=0; i<NUMBASES; i++){
		delete bases[i];
	}

	return 0;
}
