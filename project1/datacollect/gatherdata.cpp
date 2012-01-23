#include <robot_if++.h>
#include <iostream>
#include <time.h>
#include <string>
#include <math.h>
#include <stdio.h>
#include "../firfilter.h"

int zero_x;
int zero_y;
float zero_theta;
int starting_room_ID;

float tick_to_cm(int ticks){
	//1 cm ~= 45 ticks
	return ticks/45.0;
}

int cm_to_ticks(float cm){
	return 45*cm;
}

int main(int argc, char *argv[]) {
	if(argc<3){
		printf("ERROR: Invalid number of args -> should be:\n\t%s [ip/name of robot] [base filename of data files]\n", argv[0]);
		exit(-1);
	}
	char filenameNorthStarData[100];
	char filenameNorthStarDataFiltered[110];
	char filenameNorthStarDataRaw[110];
	strncpy(filenameNorthStarData,argv[2],99);
	
	char filenameWheelData[107];
	char filenameWheelDataFiltered[121];
	char filenameWheelDataRaw[121];
	strncpy(filenameWheelData,filenameNorthStarData,100);
	strcat(filenameWheelData,"_wheels");
	strcat(filenameNorthStarData,"_ns");
	
	strncpy(filenameNorthStarDataFiltered,filenameNorthStarData,110);
	strncpy(filenameNorthStarDataRaw,filenameNorthStarData,100);
	
	strncpy(filenameWheelDataFiltered,filenameWheelData,107);
	strncpy(filenameWheelDataRaw,filenameWheelData,107);
	
	strcat(filenameNorthStarDataFiltered,"_filtered.dat");
	strcat(filenameNorthStarDataRaw,"_raw.dat");
	
	strcat(filenameWheelDataFiltered,"_filtered.dat");
	strcat(filenameWheelDataRaw,"_raw.dat");
	

	if(fopen(filenameNorthStarDataFiltered, "r")!=NULL){
		printf("ERROR North Star filtered data file already exists\n");
		printf("FILE: %s\n",filenameNorthStarDataFiltered);
		exit(-2);
	}
	if(fopen(filenameNorthStarDataRaw, "r")!=NULL){
		printf("ERROR North Star raw data file already exists\n");
		printf("FILE: %s\n",filenameNorthStarDataRaw);
		exit(-3);
	}
	
	if(fopen(filenameWheelDataFiltered, "r")!=NULL) {
		printf("ERROR Wheel encoder filtered data file already exists\n");
		printf("FILE: %s\n",filenameWheelDataFiltered);
		exit(-4);
	}
	if(fopen(filenameWheelDataRaw, "r")!=NULL) {
		printf("FILE: %s\n",filenameWheelDataRaw);
		exit(-5);
	}
	
	FIRFilter *filtX = new FIRFilter(NS_X);
	FIRFilter *filtY = new FIRFilter(NS_Y);
	FIRFilter *filtTheta = new FIRFilter(NS_THETA);
	
	FIRFilter *wefiltL = new FIRFilter(WE);
	FIRFilter *wefiltR = new FIRFilter(WE);
	FIRFilter *wefiltRear = new FIRFilter(WE);
	
	printf("Battery level constants: max=%d \t home=%d \t off=%d\n",RI_ROBOT_BATTERY_MAX,RI_ROBOT_BATTERY_HOME,RI_ROBOT_BATTERY_OFF);
	
	RobotInterface *robot = new RobotInterface(argv[1],0);
	robot->reset_state();
	
	while(robot->update()!=RI_RESP_SUCCESS){
		printf("ERROR: Could not update robot\n");
		sleep(1);
	}
	
	printf("Signal Strength: %d\n",robot->NavStrengthRaw());
	switch(robot->NavStrength()){
		case RI_ROBOT_NAV_SIGNAL_STRONG:
			printf("STRONG\t%d\n",robot->NavStrength());
			break;
		case RI_ROBOT_NAV_SIGNAL_MID:
			printf("MID\t%d\n",robot->NavStrength());
			break;
		case RI_ROBOT_NAV_SIGNAL_WEAK:
			printf("WEAK\t%d\n",robot->NavStrength());
			break;
		case RI_ROBOT_NAV_SIGNAL_NO_SIGNAL:
			printf("NONE\t%d\n",robot->NavStrength());
			printf("ERROR: NO (or too little) SIGNAL\n");
			exit(-2);
			break;
	}
	
	starting_room_ID=robot->RoomID();
	
	robot->update();
	zero_x=robot->X();
	zero_y=robot->Y();
	zero_theta=robot->Theta();

	for(int i=0; i<15; i++){
		if(robot->update()== RI_RESP_SUCCESS){
			zero_x=filtX->filter((float) robot->X());
			zero_y=filtY->filter((float) robot->Y());
			zero_theta=filtTheta->filter((float) robot->Theta());
			
			wefiltL->filter((float) robot->getWheelEncoder(RI_WHEEL_LEFT));
			wefiltR->filter((float) robot->getWheelEncoder(RI_WHEEL_RIGHT));
			wefiltRear->filter((float) robot->getWheelEncoder(RI_WHEEL_REAR));
		}
		else{
			printf("ERROR: Initialization failed (%d)\n",i);
			//sleep(1);
		}
	}
	
	printf("Starting (zero'd) coordinate: %d,%d,%f\n",zero_x,zero_y,zero_theta);
	
	time_t seconds = time(NULL);
	time_t last_seconds = seconds;
	do {
		// Update the robot's sensor information
		if(robot->update() != RI_RESP_SUCCESS) {
			std::cout << "Failed to update sensor information!" << std::endl;
		}
		else{
			seconds = time(NULL);
			if (seconds < last_seconds + 1)
				continue;
			last_seconds = seconds;

			float filtx=filtX->filter((float) robot->X());
			float filty=filtY->filter((float) robot->Y());
			float filtz=filtTheta->filter((float) robot->Theta());
			
			float wfiltl=wefiltL->filter((float) robot->getWheelEncoder(RI_WHEEL_LEFT));
			float wfiltr=wefiltR->filter((float) robot->getWheelEncoder(RI_WHEEL_RIGHT));
			float wfiltrear=wefiltRear->filter((float) robot->getWheelEncoder(RI_WHEEL_REAR));
			
			// Move unless there's something in front of the robot
			if(!robot->IR_Detected()){
				if(robot->RoomID()==starting_room_ID){
					FILE * outfile=fopen(filenameNorthStarDataFiltered,"a");
					fprintf(outfile,"%f,%f,%f\n",filtx,filty,filtz);
					fflush(outfile);
					fclose(outfile);
					
					FILE * outfileraw=fopen(filenameNorthStarDataRaw,"a");
					fprintf(outfileraw,"%d,%d,%f\n",robot->X(),robot->Y(),robot->Theta());
					fflush(outfileraw);
					fclose(outfileraw);
					
					FILE * wheelfile=fopen(filenameWheelDataFiltered,"a");
					fprintf(wheelfile,"%f,%f,%f\n",wfiltl,wfiltr,wfiltrear);
					fflush(wheelfile);
					fclose(wheelfile);
					
					FILE * wheelfileraw=fopen(filenameWheelDataRaw,"a");
					fprintf(wheelfileraw,"%d,%d,%d\n",robot->getWheelEncoder(RI_WHEEL_LEFT),robot->getWheelEncoder(RI_WHEEL_RIGHT),robot->getWheelEncoder(RI_WHEEL_REAR));
					fflush(wheelfileraw);
					fclose(wheelfileraw);
					
					printf("Current Position: %d, %d, %f\n",robot->X(),robot->Y(), robot->Theta());
				}
				else{
					printf("ERROR: !!!!!!!!!!!!!!!!!!!!!!!!!!!!\n ROOM=%d\n",robot->RoomID());
				}
				//robot->Move(RI_MOVE_FORWARD, 1);
				/*
				if(0){}
				else{
					//robot->Move(RI_TURN_LEFT,5);
				}
				*/
			}
			else{
				printf("Detected IR\n");
			}
		}
	} while(1);
	
	// Clean up
	delete(robot);
	return 0;
}
