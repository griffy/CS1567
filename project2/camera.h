/**
 * camera.h
 * 
 * @brief 
 * 		This class defines the rovio's camera. It has functions for accessing, storing, 
 *      and processing the images reeturned from the camera.
 *
 * @author
 * 		Shawn Hanna
 * 		Tom Nason
 * 		Joel Griffith
 * 
 * @date
 * 		created - 3/2/2012
 * 		modified - 3/25/2012
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

// 
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
	void drawX(IplImage *image, squares_t *square, CvScalar color);
	void update();
	float centerError(int color);
	float corridorSlopeError(int color);
    regressionLine leastSquaresRegression(int color, int side);	
	float centerDistanceError(int color);
	bool onSamePlane(squares_t *leftSquare, squares_t *rightSquare);
	void calculateSlope(squares_t*, lineStruct *line);
	float findPos(squares_t* square);
	float estimatePos(squares_t* leftSquares, squares_t* rightSquares);
	squares_t* leftBiggestSquare(int color);
	squares_t* rightBiggestSquare(int color);
	squares_t* findSquaresOf(int color, int areaThreshold);
	squares_t* findSquares(IplImage *img, int areaThreshold);
	IplImage* getHSVImage();
	IplImage* getBGRImage();
	IplImage* getThresholdedImage(CvScalar low, CvScalar high);
	
	void vectorToSquares_t(std::vector<squares_t> * vec, squares_t * sqr);	//returns a list of squares starting at the pointer you give...
	void sortSquaresX(squares_t * sqr, std::vector<squares_t> * vec);		//returns a vector of squares...
	void sortSquaresY(squares_t * sqr, std::vector<squares_t> * vec);		//returns a vector of squares...
	void sortSquaresSize(squares_t * sqr, std::vector<squares_t> * vec);		//returns a vector of squares...
private:
	RobotInterface *_robotInterface;
	FIRFilter *_centerDistErrorFilter;
	FIRFilter *_slopeErrorFilter;
	int _quality;
	int _resolution;
	IplImage *_pinkThresholded;
	IplImage *_yellowThresholded;
	squares_t *_pinkSquares;
	squares_t *_yellowSquares;
	std::vector<squares_t> sqrVec;
};

#endif
