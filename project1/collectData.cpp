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

int zero_x;
int zero_y;
float zero_theta;
int starting_room_ID;
int move_flag;

int main(int argc, char *argv[]) {
    if(argc<4) {
        printf("ERROR: Invalid number of args -> should be:\n\t%s [ip/name of robot] [base filename of data files] [move binary flag 1=y]\n", argv[0]);
        exit(-1);
    }
    Robot *robot = new Robot(argv[1], 0);

    printf("Name of robot: %s\n",argv[1]);

        std::string baseFilename(argv[2]);

        move_flag = atoi(argv[3]);

        std::string pwd(getcwd(NULL,0));
        pwd+="/data_logs/";
        pwd+=+argv[2];
        pwd+="/";
        printf("Current working dir: %s\n",pwd.c_str());

        if(mkdir(pwd.c_str(),S_IREAD|S_IWRITE|S_IEXEC)!=0){
                printf("ERROR CREATING DIRECTORY FOR SAVED DATA\n");
                exit(-2);
        }

    //Generate filename
    std::string filenameNorthStarDataGlobal;
        filenameNorthStarDataGlobal=pwd+baseFilename+"_ns_global";

    //Generate filename
    std::string filenameWEdata;
        filenameWEdata=pwd+baseFilename+"_we_global";

    //Open file
    if(fopen(filenameNorthStarDataGlobal.c_str(), "r")!=NULL){
            printf("ERROR North Star filtered data file already exists\n");
            printf("FILE: %s\n",filenameNorthStarDataGlobal.c_str());
            exit(-2);
    }
    //Open file
    if(fopen(filenameWEdata.c_str(), "r")!=NULL){
            printf("ERROR WheelEncoder data file already exists\n");
            printf("FILE: %s\n",filenameWEdata.c_str());
            exit(-2);
    }

    FILE * outfileglobal=fopen(filenameNorthStarDataGlobal.c_str(),"a");
    FILE * outfilewe=fopen(filenameWEdata.c_str(),"a");

	printf("Battery: %d\n", robot->_robotInterface->Battery());
	
	

    for (int i = 0; i< 10; i++) {
        //robot->moveForward(5);
        robot->update();
		//printf(" WE %f %f %d\t\t",robot->_wePose->getTotalTheta(), robot->_wePose->getTheta(), robot->_wePose->getNumRotations());
		//printf(" NS %f %f %d\n",robot->_nsPose->getTotalTheta(), robot->_nsPose->getTheta(), robot->_nsPose->getNumRotations());
        //write to global position file
        printf("%d %f,%f,%f \n",robot->_robotInterface->RoomID(),robot->_getNSTransX(),robot->_getNSTransY(),robot->_getNSTransTheta());
        fprintf(outfileglobal,"%d %f,%f,%f %f %f\n",robot->_robotInterface->RoomID(),robot->_getNSTransX(),robot->_getNSTransY(),robot->_getNSTransTheta(), robot->_getNSHalfTransX(),robot->_getNSHalfTransY());
        //write to global position file
		printf("current pose: %f %f %f\n", robot->getPose()->getX(), robot->getPose()->getY(), robot->getPose()->getTheta());
		printf("current ns pose: \t\t\t%f %f %f\n", robot->_nsPose->getX(), robot->_nsPose->getY(), robot->_nsPose->getTheta());
		printf("current we pose: \t\t\t\t\t%f %f %f\n", robot->_wePose->getX(), robot->_wePose->getY(), robot->_wePose->getTheta());
		printf("strength: \t\t%d\n\n", robot->getStrength());
        fprintf(outfileglobal,"%d %f,%f,%f\n",robot->_robotInterface->RoomID(),robot->_getNSTransX(),robot->_getNSTransY(),robot->_getNSTransTheta());
        fprintf(outfilewe,"%d,%d,%d\n",robot->_robotInterface->getWheelEncoder(RI_WHEEL_LEFT),robot->_robotInterface->getWheelEncoder(RI_WHEEL_RIGHT),robot->_robotInterface->getWheelEncoder(RI_WHEEL_REAR));
	}

    for (int i = 0; i < 100; i++) {
        robot->turnLeft(10);
        robot->update();
        //write to global position file
        fprintf(outfileglobal,"%d %f,%f,%f \n",robot->_robotInterface->RoomID(),robot->_getNSTransX(),robot->_getNSTransY(),robot->_getNSTransTheta());
        printf("%d %f,%f,%f \n",robot->_robotInterface->RoomID(),robot->_getNSTransX(),robot->_getNSTransY(),robot->_getNSTransTheta());
        //fprintf(outfilewe,"%d,%d,%d\n",robot->_robotInterface->getWheelEncoder(RI_WHEEL_LEFT),robot->_robotInterface->getWheelEncoder(RI_WHEEL_RIGHT),robot->_robotInterface->getWheelEncoder(RI_WHEEL_REAR));
    }

    robot->stop();
    for (int i = 0; i < 10; i++) {
        robot->update();
        printf("%d %f,%f,%f \n",robot->_robotInterface->RoomID(),robot->_getNSTransX(),robot->_getNSTransY(),robot->_getNSTransTheta());
		//printf(" WE %f %f %d\t\t",robot->_wePose->getTotalTheta(), robot->_wePose->getTheta(), robot->_wePose->getNumRotations());
		//printf(" NS %f %f %d\n",robot->_nsPose->getTotalTheta(), robot->_nsPose->getTheta(), robot->_nsPose->getNumRotations());
        //write to global position file
		printf("current pose: %f %f %f\n", robot->getPose()->getX(), robot->getPose()->getY(), robot->getPose()->getTheta());
		printf("current ns pose: \t\t\t%f %f %f\n", robot->_nsPose->getX(), robot->_nsPose->getY(), robot->_nsPose->getTheta());
		printf("current we pose: \t\t\t\t\t%f %f %f\n", robot->_wePose->getX(), robot->_wePose->getY(), robot->_wePose->getTheta());
		
		
        fprintf(outfileglobal,"%d %f,%f,%f\n",robot->_robotInterface->RoomID(),robot->_getNSTransX(),robot->_getNSTransY(),robot->_getNSTransTheta());
        //fprintf(outfilewe,"%d,%d,%d\n",robot->_robotInterface->getWheelEncoder(RI_WHEEL_LEFT),robot->_robotInterface->getWheelEncoder(RI_WHEEL_RIGHT),robot->_robotInterface->getWheelEncoder(RI_WHEEL_REAR));
    }


    fflush(outfileglobal);
    fflush(outfilewe);
    fclose(outfileglobal);
    fclose(outfilewe);

    delete(robot);
    return 0;
}
