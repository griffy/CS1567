#include "robot.h"
#include "logger.h"
#include <stdio.h>

#define NUM_BASES 7

#define SPIN 0
#define MOVE_FORWARD 1
#define SIT 2

int main(int argc, char *argv[]) {
    if (argc < 3) {
        printf("ERROR: Invalid number of args -> should be:\n"
               "%s [ip/name of robot] [move flag]\n", argv[0]);
        return -1;
    }

    Robot *robot = new Robot(argv[1], 0);

    printf("Name of robot: %s\n", argv[1]);

	printf("Battery: %d\n", robot->getBattery());
	
    int moveType = atoi(argv[2]);

    for (int i = 0; i < 25; i++) {
        robot->update();

        LOG.write(LOG_LOW, "we_pose_data", 
                  "%f,%f,%f", robot->_wePose->getX(),
                              robot->_wePose->getY(),
                              robot->_wePose->getTheta());

        LOG.write(LOG_LOW, "ns_pose_data", 
                  "%f,%f,%f", robot->_nsPose->getX(),
                              robot->_nsPose->getY(),
                              robot->_nsPose->getTheta());
   
        switch (moveType) {
        case SPIN:
            robot->turnLeft(5);
            break;
        case MOVE_FORWARD:
            robot->moveForward(5);
            break;
        }
    }

    delete(robot);
    
    return 0;
}
