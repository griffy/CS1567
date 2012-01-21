#include "robot.h"

#include <string>

int main() {
    Robot *robot = new Robot("192.168.1.42", 0);

    delete(robot);
    return 0;
}