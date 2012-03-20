#include "robot.h"
#include "logger.h"
#include "camera.h"
#include <stdio.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <robot_color.h>


#define NUM_BASES 7

void drawX(IplImage *image, squares_t *square, CvScalar color) {
	if (square == NULL) {
		return;
	}
	
	CvPoint pt1, pt2;

	// Draw an X marker on the image
	int sqAmt = (int) (sqrt(square->area) / 2);	

	// Upper Left to Lower Right
	pt1.x = square->center.x - sqAmt;
	pt1.y = square->center.y - sqAmt;
	pt2.x = square->center.x + sqAmt;
	pt2.y = square->center.y + sqAmt;
	cvLine(image, pt1, pt2, color, 3, CV_AA, 0);

	// Lower Left to Upper Right
	pt1.x = square->center.x - sqAmt;
	pt1.y = square->center.y + sqAmt;
	pt2.x = square->center.x + sqAmt;
	pt2.y = square->center.y - sqAmt;
	cvLine(image, pt1, pt2, color, 3, CV_AA, 0);
}

int main(int argc, char *argv[]) {
	if (argc < 2) {
		printf("ERROR: need argument for robot name\n");
		return -1;
	}

    LOG.setImportanceLevel(LOG_LOW);

	Robot *robot = new Robot(argv[1], 0);

	cvNamedWindow("BGR Image", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("Thresholded Image", CV_WINDOW_AUTOSIZE);

	/*
	while (true) {
		robot->update();

		IplImage *bgr = robot->_camera->getBGRImage();
		IplImage *thresholded = robot->_camera->getThresholdedImage(RC_PINK_LOW, RC_PINK_HIGH);

		drawX(bgr, robot->_camera->leftBiggestSquare(COLOR_PINK), RED);
		drawX(bgr, robot->_camera->rightBiggestSquare(COLOR_PINK), GREEN);
		
		LOG.printfScreen(LOG_LOW, "screenError", "Center error: %d\n", robot->_camera->centerDistanceError(COLOR_PINK));

		cvShowImage("BGR Image", bgr);
		cvShowImage("Thresholded Image", thresholded);

		//Line regression test
		robot->_camera->corridorSlopeError(COLOR_PINK);

		cvWaitKey(0);

		cvReleaseImage(&bgr);
		cvReleaseImage(&thresholded);
	}
*/

	robot->move(DIR_EAST, 5);
	robot->turn(DIR_RIGHT, DEGREE_90);
	robot->move(DIR_SOUTH, 3);

	delete robot;


/*
    // Base locations in cm within the global coordinate system
	Pose * bases[NUM_BASES];
	bases[0] = new Pose(300, 0, 0); // base 1
	bases[1] = new Pose(229, 183, 0); // base 2
    bases[2] = new Pose(326, 183, 0); // fake base
	bases[3] = new Pose(392, 300, 0); // base 3
    bases[4] = new Pose(318, 386, 0); // fake base
    bases[5] = new Pose(49, 386, 0); // base 4
    bases[6] = new Pose(0, 0, 0); // base 5/0

    for (int i = 0; i < NUM_BASES; i++) {
    	printf("moving to base %d...\n", i+1);
    	robot->moveTo(bases[i]->getX(), bases[i]->getY());
    	printf("reached base %d!\n", i+1);
    }

    printf("done!\n");

	delete robot;
	for (int i = 0; i < NUM_BASES; i++) {
		delete bases[i];
	}
*/
	return 0;
}
