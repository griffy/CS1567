#include <robot_if++.h>
#include <iostream>
#include <string>
#include <math.h>
#include <stdio.h>

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

int main(int argv, char **argc) {
	
	
	RobotInterface *robot = new RobotInterface(argc[1],0);
	robot->reset_state();
	
	robot->update();
	
	zero_x=robot->X();
	zero_y=robot->Y();
	
	int avg[5];
	avg[0]=robot->X()-zero_x;
	avg[1]=avg[0];
	avg[2]=avg[0];
	avg[3]=avg[0];
	avg[4]=avg[0];
	int cursor=0;
	
	// Make sure we have a valid command line argument
	if(argv <= 1) {
		std::cout << "Usage: robot_test <address of robot>" << std::endl;
		exit(-1);
	}
	
	// Setup the robot interface
	
	// Update the robot's sensor information
	if(robot->update() != RI_RESP_SUCCESS) {
		std::cout << "Failed to update sensor information!" << std::endl;
		exit(-2);
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
			printf("ERROR: NO SIGNAL\n");
			exit(-2);
			break;
	}
	
	zero_theta=robot->Theta();
	starting_room_ID=robot->RoomID();
	
	printf("Starting (zero'd) coordinate: %d,%d,%f\n",zero_x,zero_y,zero_theta);
	
	// Action loop
	
	do {
		// Update the robot's sensor information
		if(robot->update() != RI_RESP_SUCCESS) {
			std::cout << "Failed to update sensor information!" << std::endl;
			//break;
		}
		else{
			avg[cursor++%5]=robot->X()-zero_x;
			
			printf("Current Position: %d, %d, %f\n",robot->X(),robot->Y(), robot->Theta());
			// Move forward unless there's something in front of the robot
			if(!robot->IR_Detected()){
				//move forward 3 meters
				
				FILE * outfile=fopen("drive3mstop.dat","a");
				fprintf(outfile,"%d,%d,%f\n",robot->X(),robot->Y(),robot->Theta());
				fflush(outfile);
				fclose(outfile);
				
				FILE * wheelfile=fopen("drive3mstopwheels.dat","a");
				fprintf(wheelfile,"%d,%d,%d\n",robot->getWheelEncoder(RI_WHEEL_LEFT),robot->getWheelEncoder(RI_WHEEL_RIGHT),robot->getWheelEncoder(RI_WHEEL_REAR));
				fflush(wheelfile);
				fclose(wheelfile);
				
				// 			int sum=0;
				// 			for(int i=0; i<5; i++){
					// 			sum+=avg[i];
				// 			}
				// 			float average=sum/5.0;
				// 			
				/*if(average>=cm_to_ticks(300)) {
					printf("DONE");
					printf("\t\t\tCurr distance: avg=%f",average);
					printf(" in cm: %f\n",tick_to_cm(average));
					break;
				}*/
				if(0){}
				else{
					robot->Move(RI_MOVE_FORWARD,5);
// 					printf("\t\t\tCurr distance: avg=%f",average);
// 					printf(" in cm: %f\n",tick_to_cm(average));
				}
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