#include "robot.h"
#include "logger.h"
#include <stdio.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <robot_color.h>

int main(int argc, char *argv[]) {
	if (argc < 2) {
		printf("ERROR: need argument for robot name\n");
		return -1;
	}

    LOG.setImportanceLevel(LOG_OFF);

	Robot *robot = new Robot(argv[1], 0);

	cvNamedWindow("BGR Image", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("Thresholded Image", CV_WINDOW_AUTOSIZE);

	robot->update();
	IplImage *bgr = robot->_camera->getBGRImage();
	IplImage *thresholded = robot->_camera->getThresholdedImage(RC_PINK_LOW, RC_PINK_HIGH);
	cvShowImage("BGR Image", bgr);
	cvShowImage("Thresholded Image", thresholded);

	cvWaitKey(0);

	cvReleaseImage(&bgr);
	cvReleaseImage(&thresholded);

	delete robot;

	return 0;
}
