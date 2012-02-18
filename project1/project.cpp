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
	bases[0] = new Pose(340, 0, 0); // base 1
    //bases[4] = new Pose(340, 183, 0); // fake base between 1 & two
	bases[1] = new Pose(300, 183, 0); // base 2
    bases[2] = new Pose(326, 183, 0); // fake base
	bases[3] = new Pose(320, 415, 0); // base 3
    bases[4] = new Pose(318, 386, 0); // fake base
    bases[5] = new Pose(49, 386, 0); // base 4
    bases[6] = new Pose(0, 0, 0); // base 5/0

	Robot *robot = new Robot(argv[1], 0);

	printf("battery: %d\n", robot->_robotInterface->Battery());

    for (int i = 0; i < 7; i++) {
    	printf("moving to base %d...\n", i+1);
    	robot->moveTo(bases[i]->getX(), bases[i]->getY());
    	printf("\n\n\n\n\n\n\n\n\n\n\n\n\n\nreached base %d!\n\n\n\n\n\n\n\n\n\n\n\n\n\n", i+1);

		printf("prefilling data\n");
		for (int j = 0; j < 5; j++) {
			robot->update();
		}
    }

    printf("\n\ndone!\n\n");

	delete robot;
	for (int i = 0; i < NUMBASES; i++) {
		delete bases[i];
	}

	return 0;
}
