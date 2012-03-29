/**
 * camera.h
 * 
 * @brief 
 * 		This class defines the rovio's camera. It has functions for accessing, storing, 
 *      and processing the images returned from the camera.
 *
 * @author
 * 		Shawn Hanna
 * 		Tom Nason
 * 		Joel Griffith
 *
 **/

#ifndef CS1567_CAMERA_H
#define CS1567_CAMERA_H

#include <stdio.h>
#include <vector>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <robot_if++.h>
#include <robot_color.h>

#include "fir_filter.h"

// constants used by the constructor as defaults
// for setting up the camera
#define CAMERA_QUALITY RI_CAMERA_QUALITY_HIGH
#define CAMERA_RESOLUTION RI_CAMERA_RES_320

// constants for differentiating what colors to threshold in camera
#define COLOR_PINK 0
#define COLOR_YELLOW 1

// constants for what side of an image to process
#define SIDE_LEFT 0
#define SIDE_RIGHT 1

// smallest acceptable square to pick up in image processing
#define DEFAULT_SQUARE_SIZE 50 // in pixels

// largest allowable slope between centers of two largest squares
// to decide if they're on the same plane or not
#define MAX_PLANE_SLOPE 10 // in pixels

// low and high colors for thresholding
#define YELLOW_LOW cvScalar(25, 100, 100)
#define YELLOW_HIGH cvScalar(35, 255, 255)
#define PINK_LOW cvScalar(160, 85, 85)
#define PINK_HIGH cvScalar(179, 255, 255)
// since pink wraps around, we also need to threshold red and or with pink
#define RED_LOW cvScalar(0, 85, 85)
#define RED_HIGH cvScalar(7, 255, 255)

// constants for colors to use for drawing over images
#define RED CV_RGB(255, 0, 0)
#define GREEN CV_RGB(0, 255, 0)
#define BLUE CV_RGB(0, 0, 255)

// define the min and max allowable slopes for lines of regression
// through found squares
#define MAX_SLOPE 2.5
#define MIN_SLOPE 0.01

// more specific slope limits for lines of regression based on 
// empirical data
#define RIGHT_LEFT_SLOPE -0.35
#define RIGHT_RIGHT_SLOPE -2.22
#define RIGHT_MIDDLE_SLOPE -0.67

#define LEFT_LEFT_SLOPE 2.6
#define LEFT_RIGHT_SLOPE 0.55
#define LEFT_MIDDLE_SLOPE 0.7

// the largest allowable difference between the slopes of two
// lines of regression before deciding the slope error is at its max
#define MAX_SLOPE_DIFFERENCE 0.5

// how many images should be taken and processed to find (average)
// center error 
#define NUM_CAMERA_ERRORS 7

// define a line for regression calculations to utilize slope error
typedef struct regLine {
	float intercept;
	float slope;
	int numSquares;
} regressionLine;

class Camera {
public:
	Camera(RobotInterface *robotInterface);
	~Camera();
	void setQuality(int quality);
	void setResolution(int resolution);
	void markSquare(IplImage *image, squares_t *square, CvScalar color);
	void update();
	float centerError(int color);
	float centerDistanceError(int color);
	float corridorSlopeError(int color);
    regressionLine leastSquaresRegression(int color, int side);	
	bool onSamePlane(squares_t *leftSquare, squares_t *rightSquare);
	squares_t* biggestSquare(int color, int side);
	int squareCount(int color, int side);
	IplImage* thresholdedOf(int color);
	squares_t* squaresOf(int color);
	squares_t* findSquaresOf(int color, int areaThreshold);
	squares_t* findSquares(IplImage *img, int areaThreshold);
	IplImage* getHSVImage();
	IplImage* getBGRImage();
	IplImage* getThresholdedImage(CvScalar low, CvScalar high);
private:
	RobotInterface *_robotInterface;
	int _quality;
	int _resolution;
	IplImage *_pinkThresholded;
	IplImage *_yellowThresholded;
	squares_t *_pinkSquares;
	squares_t *_yellowSquares;
};

#endif
