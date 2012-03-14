#include "robot.h"
#include "logger.h"
#include <stdio.h>


int main(int argc, char *argv[]) {
	if (argc < 2) {
		printf("ERROR: need argument for robot name\n");
		return -1;
	}

    LOG.setImportanceLevel(LOG_OFF);

	Robot *robot = new Robot(argv[1], 0);

	printf("battery: %d\n", robot->getBattery());

	CameraImage image;
	while(1){
		if(!robot->update()){
			print("Error, could not update\n");
			continue;
		}
		robot->getImage();
		image = new CameraImage();
		cvShowImage(image.getImage());
		delete(image);
	}

	delete robot;
	return 0;
}
