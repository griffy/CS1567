#include "../logger.h"
#include <robot_if++.h>
#include <robot_color.h>
#include <iostream>
#include <string>

// int brightness, int contrast, int framerate, int resolution, int quality
/*
// 176x144
#define RI_CAMERA_RES_176		0
// 320x240
#define RI_CAMERA_RES_320		1
// 352x240                          <<<<< This one causes a segfault
#define RI_CAMERA_RES_352		2
// 640x480
#define RI_CAMERA_RES_640		3
*/
/*
// Camera quality
#define RI_CAMERA_QUALITY_LOW		0		
#define RI_CAMERA_QUALITY_MID		1
#define RI_CAMERA_QUALITY_HIGH		2
#define RI_CAMERA_MAX_IMG_SIZE		(1024*1024)
*/
    
#define QUALITY RI_CAMERA_QUALITY_HIGH
#define RESOLUTION RI_CAMERA_RES_640

int main(int argv, char **argc) {
	int major, minor;
	IplImage *image = NULL, *hsv = NULL, *threshold = NULL;
	squares_t *squares, *biggest, *sq_idx;
	CvPoint pt1, pt2;
	int sq_amt;

	// Make sure we have a valid command line argument
	if(argv <= 1) {
		std::cout << "Usage: robot_test <address of robot>" << std::endl;
		exit(-1);
	}

	// Setup the robot interface
	RobotInterface *robot = new RobotInterface(argc[1], 0);
	
	// Setup the camera
	if(robot->CameraCfg(RI_CAMERA_DEFAULT_BRIGHTNESS, 
	                    RI_CAMERA_DEFAULT_CONTRAST, 
	                    5, 
	                    RESOLUTION, 
	                    QUALITY)) {
		std::cout << "Failed to configure the camera!" << std::endl;
		exit(-1);
	}
	
	CvSize size;
	switch (RESOLUTION) {
	case RI_CAMERA_RES_640:
	    size = cvSize(640, 480);
	    break;
	case RI_CAMERA_RES_352:
	    size = cvSize(352, 240);
	    break;
	case RI_CAMERA_RES_320:
	    size = cvSize(320, 240);
	    break;
	case RI_CAMERA_RES_176:
	    size = cvSize(176, 144);
	    break;
	}
	// Create an image to store the image from the camera
	image = cvCreateImage(size, IPL_DEPTH_8U, 3);

	// Create an image to store the HSV version in
	hsv = cvCreateImage(size, IPL_DEPTH_8U, 3);

	// Move the head up to the middle position
	robot->Move(RI_HEAD_MIDDLE, RI_FASTEST);

    char rgbStr[80];
    char hsvStr[80];
    sprintf(rgbStr, "rgb_%d_%d", QUALITY, RESOLUTION);
    sprintf(hsvStr, "hsv_%d_%d", QUALITY, RESOLUTION);
    std::string rgbName(rgbStr);
    std::string hsvName(hsvStr);
        
	time_t seconds = time(NULL);
	time_t last_seconds = seconds;
	// Action loop
	do {
		// Update the robot's sensor information
		if(robot->update() != RI_RESP_SUCCESS) {
			std::cout << "Failed to update sensor information!" << std::endl;
			continue;
		}
		
		// Get the current camera image
		if(robot->getImage(image) != RI_RESP_SUCCESS) {
			std::cout << "Unable to capture an image!" << std::endl;
			continue;
		}
		
		// Convert the image from RGB to HSV
		cvCvtColor(image, hsv, CV_BGR2HSV);

        // get the current time in secs
        seconds = time(NULL);
        // if a second has passed, save the image
        if (seconds >= last_seconds + 5) {
            // add a "header" specifying the size
            LOG.printfFile(LOG_HIGH, rgbName, "%d\n", image->imageSize);
            LOG.printfFile(LOG_HIGH, hsvName, "%d\n", hsv->imageSize);
            // print out the contents
            for (int i = 0; i < image->imageSize; i += 3) {
                LOG.printfFile(LOG_HIGH, rgbName, 
                               "%d,%d,%d\n", 
                               image->imageData[i], // channel 1 (b)
                               image->imageData[i+1], // channel 2 (g)
                               image->imageData[i+2]); // channel 3 (r)
            }
            for (int i = 0; i < hsv->imageSize; i += 3) {
                LOG.printfFile(LOG_HIGH, hsvName, 
                               "%d,%d,%d\n", 
                               hsv->imageData[i],
                               hsv->imageData[i+1],
                               hsv->imageData[i+2]);
            }
            LOG.flushFile(rgbName);
            LOG.flushFile(hsvName);
            
            last_seconds = seconds;
        }
        else {
            printf("You have %d second(s) to move me!\n", (last_seconds+5)-seconds);
        }
		// Move forward unless there's something in front of the robot
//		if(!robot->IR_Detected())
//			robot->Move(RI_MOVE_FORWARD, RI_SLOWEST);
	} while(1);	

	// Clean up (although we'll never get here...)
	delete(robot);
	
	// Free the images
	cvReleaseImage(&hsv);
	cvReleaseImage(&image);

	return 0;
}

