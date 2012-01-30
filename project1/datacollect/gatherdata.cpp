#include <robot_if++.h>
#include <iostream>
#include <time.h>
#include <sys/time.h>
#include <string>
#include <math.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
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
	printf("Name of robot: %s\n",argv[1]);
	
	std::string baseFilename(argv[2]);
	
	std::string pwd(getcwd(NULL,0));
	pwd+="/data_logs/";
	pwd+=+argv[2];
	pwd+="/";
	printf("Current working dir: %s\n",pwd.c_str());
	
	if(mkdir(pwd.c_str(),S_IREAD|S_IWRITE|S_IEXEC)!=0){
		printf("ERROR CREATING DIRECTORY FOR SAVED DATA\n");
		exit(-2);
	}
	
	std::string filenameNorthStarData(pwd);
	std::string filenameNorthStarDataFiltered;
	filenameNorthStarDataFiltered=pwd+baseFilename+"_ns_filtered";
	std::string filenameNorthStarDataRaw;
	filenameNorthStarDataRaw=pwd+baseFilename+"_ns_raw";
	
	std::string filenameWheelData(pwd);
	std::string filenameWheelDataFiltered;
	filenameWheelDataFiltered=pwd+baseFilename+"_we__filtered";
	std::string filenameWheelDataRaw;
	filenameWheelDataRaw=pwd+baseFilename+"_we__raw";
	
	std::string filenameTimeData(pwd);
	filenameTimeData=pwd+baseFilename+"_time";
	
	std::string filenameSignalStrength(pwd);
	filenameSignalStrength=pwd+baseFilename+"_signal";
	
	std::string filenameRoomData(pwd);
	filenameRoomData=pwd+baseFilename+"_room";

	if(fopen(filenameNorthStarDataFiltered.c_str(), "r")!=NULL){
		printf("ERROR North Star filtered data file already exists\n");
		printf("FILE: %s\n",filenameNorthStarDataFiltered.c_str());
		exit(-2);
	}
	if(fopen(filenameNorthStarDataRaw.c_str(), "r")!=NULL){
		printf("ERROR North Star raw data file already exists\n");
		printf("FILE: %s\n",filenameNorthStarDataRaw.c_str());
		exit(-3);
	}
	
	if(fopen(filenameWheelDataFiltered.c_str(), "r")!=NULL) {
		printf("ERROR Wheel encoder filtered data file already exists\n");
		printf("FILE: %s\n",filenameWheelDataFiltered.c_str());
		exit(-4);
	}
	if(fopen(filenameWheelDataRaw.c_str(), "r")!=NULL) {
		printf("FILE: %s\n",filenameWheelDataRaw.c_str());
		exit(-5);
	}
	if(fopen(filenameTimeData.c_str(), "r")!=NULL) {
		printf("FILE: %s\n",filenameTimeData.c_str());
		exit(-5);
	}
	if(fopen(filenameSignalStrength.c_str(), "r")!=NULL) {
		printf("FILE: %s\n",filenameSignalStrength.c_str());
		exit(-5);
	}
	if(fopen(filenameRoomData.c_str(), "r")!=NULL) {
		printf("FILE: %s\n",filenameRoomData.c_str());
		exit(-5);
	}
	
	printf("Successfully named log files\n");
	printf("Northstar full path: %d\n",filenameNorthStarData.c_str());
	
	FIRFilter *filtX = new FIRFilter("../filters/ns_x.ffc");
	FIRFilter *filtY = new FIRFilter("../filters/ns_y.ffc");
	FIRFilter *filtTheta = new FIRFilter("../filters/ns_theta.ffc");
	
	FIRFilter *wefiltL = new FIRFilter("../filters/we.ffc");
	FIRFilter *wefiltR = new FIRFilter("../filters/we.ffc");
	FIRFilter *wefiltRear = new FIRFilter("../filters/we.ffc");
	
	
	RobotInterface *robot = new RobotInterface(argv[1],0);
	robot->reset_state();
	while(robot->update()!=RI_RESP_SUCCESS){
		printf("ERROR: Could not update robot\n");
		sleep(1);
	}
	
	printf("Battery level constants: max=%d \t home=%d \t off=%d\n",RI_ROBOT_BATTERY_MAX,RI_ROBOT_BATTERY_HOME,RI_ROBOT_BATTERY_OFF);
	printf("Battery Level: %d\n",robot->Battery());
	
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
    /*	
	time_t seconds = time(NULL);
	time_t last_seconds = seconds;
    */
	
	timeval tv;
	tv.tv_usec=0;
	tv.tv_sec=0;
	gettimeofday(&tv,0);
	long startTime=(1000000*tv.tv_sec+tv.tv_usec)/1000;
	
	do {
		// Update the robot's sensor information
		if(robot->update() != RI_RESP_SUCCESS) {
			std::cout << "Failed to update sensor information!" << std::endl;
		}
		else{
			gettimeofday(&tv,0);
            long time=1000000*tv.tv_sec+tv.tv_usec;
			time=time/1000;
			
			// write to time file
			FILE * timefile=fopen(filenameTimeData.c_str(),"a");
			fprintf(timefile,"%ld\n",time);
			fflush(timefile);
			fclose(timefile);
			
			// write to signal strength file
			FILE * signalfile=fopen(filenameSignalStrength.c_str(),"a");
			fprintf(signalfile,"%d\n",robot->NavStrengthRaw());
			fflush(signalfile);
			fclose(signalfile);
			
			// write to room file
			FILE * roomfile=fopen(filenameRoomData.c_str(),"a");
			fprintf(roomfile,"%d\n",robot->RoomID());
			fflush(roomfile);
			fclose(roomfile);
			
			float filtx=filtX->filter((float) robot->X());
			float filty=filtY->filter((float) robot->Y());
			float filtz=filtTheta->filter((float) robot->Theta());
			
			float wfiltl=wefiltL->filter((float) robot->getWheelEncoder(RI_WHEEL_LEFT));
			float wfiltr=wefiltR->filter((float) robot->getWheelEncoder(RI_WHEEL_RIGHT));
			float wfiltrear=wefiltRear->filter((float) robot->getWheelEncoder(RI_WHEEL_REAR));
			
			// Move unless there's something in front of the robot
			//if(!robot->IR_Detected()){
			FILE * outfile=fopen(filenameNorthStarDataFiltered.c_str(),"a");
			fprintf(outfile,"%f,%f,%f\n",filtx,filty,filtz);
			fflush(outfile);
			fclose(outfile);
			
			FILE * outfileraw=fopen(filenameNorthStarDataRaw.c_str(),"a");
			fprintf(outfileraw,"%d,%d,%f\n",robot->X(),robot->Y(),robot->Theta());
			fflush(outfileraw);
			fclose(outfileraw);
			
			FILE * wheelfile=fopen(filenameWheelDataFiltered.c_str(),"a");
			fprintf(wheelfile,"%f,%f,%f\n",wfiltl,wfiltr,wfiltrear);
			fflush(wheelfile);
			fclose(wheelfile);
			
			FILE * wheelfileraw=fopen(filenameWheelDataRaw.c_str(),"a");
			fprintf(wheelfileraw,"%d,%d,%d\n",robot->getWheelEncoder(RI_WHEEL_LEFT),robot->getWheelEncoder(RI_WHEEL_RIGHT),robot->getWheelEncoder(RI_WHEEL_REAR));
			fflush(wheelfileraw);
			fclose(wheelfileraw);
			
			printf("Current Position (room %d): %d, %d, %f\n",robot->RoomID(),robot->X(),robot->Y(), robot->Theta());
			printf("\t\t\t\t\t\tFiltered Position Position (room %d): %f, %f, %f\n",robot->RoomID(),filtx,filty,filtz);
			//robot->Move(RI_TURN_LEFT, 1);
			/*
			if(0){}
			else{
				//robot->Move(RI_TURN_LEFT,5);
			}
			*/
			//}
			//else{
			//	printf("Detected IR\n");
			//}
		}
	} while(1);
	
	// Clean up
	delete(robot);
	return 0;
}
