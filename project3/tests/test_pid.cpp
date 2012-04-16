#include "../PID.h"
#include <stdio.h>

#define PID_KP 0.8
#define PID_KI 0.25
#define PID_KD 0.30

#define MIN_ERROR -1.0
#define MAX_ERROR 1.0

int main() {
    float error, gain, speed;

    PIDConstants constants = {PID_KP, PID_KI, PID_KD};
    PID *pid = new PID(&constants, MIN_ERROR, MAX_ERROR);

    error = MAX_ERROR;
    for (int i = 0; i < 10; i++) {
        pid->updatePID(error);
    }

    for (int i = 0; i < 10; i++) {
        gain = pid->updatePID(error);
        speed = 10 - 9 * gain;

        printf("Error: %f\t Gain: %f\t Speed: %f\n",
               error, gain, speed);

        error -= 0.1;
    }

    return 0;
}