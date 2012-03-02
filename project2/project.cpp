#include "robot.h"
#include "logger.h"
#include <stdio.h>

#define NUM_BASES 7

int main(int argc, char *argv[]) {
	if (argc < 2) {
		printf("ERROR: need argument for robot name\n");
		return -1;
	}

    LOG.setImportanceLevel(LOG_OFF);

    // Base locations in cm within the global coordinate system
	Pose * bases[NUM_BASES];
	bases[0] = new Pose(340, 0, 0); // base 1
	bases[1] = new Pose(229, 183, 0); // base 2
    bases[2] = new Pose(326, 163, 0); // fake base, should be 183 y
	bases[3] = new Pose(392, 300, 0); // base 3
    bases[4] = new Pose(318, 386, 0); // fake base
    bases[5] = new Pose(49, 386, 0); // base 4
    bases[6] = new Pose(0, 0, 0); // base 5/0

	Robot *robot = new Robot(argv[1], 0);

	printf("battery: %d\n", robot->_robotInterface->Battery());

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

	return 0;
}
