#include "fakerobotinterface.h"
#include "../../constants.h"
#include "../../firfilter.h"
#include "../../utilities.h"

#include <robot_if++.h>
#include <cstdio>
#include <cmath>

float deltaX(float left, float right, float rear) {
    return (left * cos(DEGREE_150) + 
            right * cos(DEGREE_30) +
            rear) / 3;
}

float deltaY(float left, float right) {
    return (left * sin(DEGREE_150) + right * sin(DEGREE_30)) / 2;
}

void printWE(FakeRobotInterface *robot) {
    float totalX = 0;
    float totalY = 0;
    robot->reset();
    while (robot->update() == RI_RESP_SUCCESS) {
        int weLeft = robot->getWheelEncoder(RI_WHEEL_LEFT);
        int weRight = robot->getWheelEncoder(RI_WHEEL_RIGHT);
        int weRear = robot->getWheelEncoder(RI_WHEEL_REAR);
        totalX += deltaX((float)weLeft, (float)weRight, (float)weRear);
        totalY += deltaY((float)weLeft, (float)weRight);
        printf("%d,%d,%d::(%f,%f)\n", weLeft, weRight, weRear, 
                                      totalX, totalY);
    }
}

void printWEFiltered(FakeRobotInterface *robot) {
    FIRFilter *weLeftFilter = new FIRFilter("../../filters/we.ffc");
    FIRFilter *weRightFilter = new FIRFilter("../../filters/we.ffc");
    FIRFilter *weRearFilter = new FIRFilter("../../filters/we.ffc");
    
    float totalX = 0;
    float totalY = 0;
    robot->reset();
    while (robot->update() == RI_RESP_SUCCESS) {
        int weLeft = robot->getWheelEncoder(RI_WHEEL_LEFT);
        int weRight = robot->getWheelEncoder(RI_WHEEL_RIGHT);
        int weRear = robot->getWheelEncoder(RI_WHEEL_REAR);
        
        float weLeftF = weLeftFilter->filter((float)weLeft);
        float weRightF = weRightFilter->filter((float)weRight);
        float weRearF = weRearFilter->filter((float)weRear);

        totalX += deltaX(weLeftF, weRightF, weRearF);
        totalY += deltaY(weLeftF, weRightF);
        printf("%.0f,%.0f,%.0f::(%f,%f)\n", weLeftF, weRightF, weRearF, 
                                            totalX, totalY);
    }

    delete weLeftFilter;
    delete weRightFilter;
    delete weRearFilter;
}

void printNS(FakeRobotInterface *robot) {
    robot->reset();
    while (robot->update() == RI_RESP_SUCCESS) {
        int nsX = robot->X();
        int nsY = robot->Y();
        int nsTheta = robot->Theta();
        
        printf("%d,%d,%d\n", nsX, nsY, nsTheta);
    }
}

void printNSFiltered(FakeRobotInterface *robot) {
    FIRFilter *nsXFilter = new FIRFilter("../../filters/ns_x.ffc");
    FIRFilter *nsYFilter = new FIRFilter("../../filters/ns_y.ffc");
    FIRFilter *nsThetaFilter = new FIRFilter("../../filters/ns_theta.ffc");

    robot->reset();
    while (robot->update() == RI_RESP_SUCCESS) {
        int nsX = robot->X();
        int nsY = robot->Y();
        int nsTheta = robot->Theta();
        
        float nsXF = nsXFilter->filter((float)nsX);
        float nsYF = nsYFilter->filter((float)nsY);
        float nsThetaF = nsThetaFilter->filter((float)nsTheta);

        printf("%.0f,%.0f,%.0f\n", nsXF, nsYF, nsThetaF);
    }

    delete nsXFilter;
    delete nsYFilter;
    delete nsThetaFilter;
}

int main() {
    FakeRobotInterface *robot = new FakeRobotInterface("192.168.1.42", 0);

    printf("Wheel Encoder Data\n");    
    printWE(robot);
    printf("\n");
    printf("Filtered Wheel Encoder Data\n");    
    printWEFiltered(robot);
    printf("\n");
    printf("North Star Data\n");    
    printNS(robot);
    printf("\n");
    printf("Filtered North Star Data\n");    
    printNSFiltered(robot);

    delete robot;
    return 0;
}
