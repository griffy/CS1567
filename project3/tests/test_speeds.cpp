#include "../robot.h"
#include <stdio.h>

int main(int argc, char *argv[]) {
    if (argc < 2) {
        printf("ERROR: need argument for robot name\n");
        return -1;
    }

    //Robot *robot = new Robot(argv[1], 0);
    RobotInterface *robotInterface = new RobotInterface(argv[1], 0);
    while (true) {
        robotInterface->Move(RI_MOVE_FORWARD, 1);
    }
    return 0;
}