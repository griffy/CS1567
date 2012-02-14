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
	bases[0] = new Pose(0, 0, 0);
	bases[1] = new Pose(341, 0, 0);
	bases[2] = new Pose(240, 186, 0);
	bases[3] = new Pose(320, 186, 0);
	bases[4] = new Pose(402, 303, 0);
	bases[5] = new Pose(402, 353, 0);
	bases[6] = new Pose(69, 419, 0);

	Robot *robot = new Robot(argv[1], 0);

	printf("prefilling data\n");
	for (int i = 0; i < 15; i++) {
		robot->update();
	}
	
	printf("Get battery: %d\n", robot->_robotInterface->Battery());

    for (int i = 0; i < NUMBASES; i) {
    	//printf("moving to base %d", i+1);
        /*if(!robot->moveToFull(bases[i]->getX(),bases[i]->getY())) {
			robot->printFailureDialog();
		}
		*/
		robot->update();

            printf("X global: %f\t\tY global: %f\t\tTheta global: %f\n",
                robot->_nsPose->getX(),
                robot->_nsPose->getY(),
                robot->_nsPose->getTheta());
            printf("X raw: %d\t\tY raw: %d\t\tTheta raw: %f\n",robot->_robotInterface->X(),robot->_robotInterface->Y(),robot->_robotInterface->Theta());
			sleep(0.1);
    }

    printf("done!");

	delete [] bases;
	
	delete robot;

	return 0;
}
