#include "robot.h"
#include "logger.h"
#include "camera.h"
#include <stdio.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <robot_color.h>

int main(int argc, char *argv[]) {
	if (argc < 2) {
		printf("ERROR: need argument for robot name\n");
		return -1;
	}

    LOG.setImportanceLevel(LOG_LOW);

	Robot *robot = new Robot(argv[1], 0);

	cvNamedWindow("BGR Image", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("Thresholded Image", CV_WINDOW_AUTOSIZE);

	while(1) {
		robot->update();
		IplImage *bgr = robot->_camera->getBGRImage();
		IplImage *thresholded = robot->_camera->getThresholdedImage(RC_PINK_LOW, RC_PINK_HIGH);

		LOG.printfScreen(LOG_LOW, "screenError", "Left error: %d\n", robot->_camera->leftSquareDistanceError(COLOR_PINK));
		LOG.printfScreen(LOG_LOW, "screenError", "Right error: %d\n", robot->_camera->rightSquareDistanceError(COLOR_PINK));
		LOG.printfScreen(LOG_LOW, "screenError", "Center error: %d\n", robot->_camera->centerDistanceError(COLOR_PINK));

		cvShowImage("BGR Image", bgr);
		cvShowImage("Thresholded Image", thresholded);

		cvWaitKey(0);

		cvReleaseImage(&bgr);
		cvReleaseImage(&thresholded);
	}

	delete robot;

	return 0;
}
