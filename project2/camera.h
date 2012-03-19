#ifndef CS1567_CAMERA_H
#define CS1567_CAMERA_H

#include "fir_filter.h"
#include <stdio.h>
#include <vector>
#include <opencv/cv.h>
#include <robot_if++.h>

#define COLOR_PINK 0
#define COLOR_YELLOW 1

#define SIDE_LEFT 0
#define SIDE_RIGHT 1

#define DEFAULT_SQUARE_SIZE 1
#define MAX_PLANE_SLOPE 5 // in pixels

#define PINK_LOW cvScalar(160, 100, 100)
#define PINK_HIGH cvScalar(180, 255, 255)

#define YELLOW_LOW cvScalar(20, 100, 100)
#define YELLOW_HIGH cvScalar(30, 255, 255)

typedef struct regLine {
	float intercept;
	float slope;
	int numSquares;
} regressionLine;

class Camera {
public:
	
	struct lineStruct {
		float slope, yInt, r2;
	};
	
	Camera(RobotInterface *robotInterface);
	~Camera();
	void setQuality(int quality);
	void setResolution(int resolution);
	void update();
	int corridorSlopeError(int color);
        regressionLine leastSquaresRegression(int color, int side);	
	int centerDistanceError(int color);
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
	FIRFilter *_leftDistanceErrorFilter;
	FIRFilter *_rightDistanceErrorFilter;
	int _quality;
	int _resolution;
	IplImage *_pinkThresholded;
	IplImage *_yellowThresholded;
	squares_t *_pinkSquares;
	squares_t *_yellowSquares;
	std::vector<squares_t> sqrVec;
};

#endif
