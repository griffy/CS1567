#ifndef CS1567_CAMERA_H
#define CS1567_CAMERA_H

#include "fir_filter.h"
#include <opencv/cv.h>
#include <robot_if++.h>

#define COLOR_PINK 0
#define COLOR_YELLOW 1

#define DEFAULT_SQUARE_SIZE 250

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
	int centerDistanceError(int color);
	int leftSquareDistanceError(int color);
	int rightSquareDistanceError(int color);
	squares_t* findSquaresOf(int color, int areaThreshold);
	squares_t* findSquares(IplImage *img, int areaThreshold);
	IplImage* getHSVImage();
	IplImage* getBGRImage();
	IplImage* getThresholdedImage(CvScalar low, CvScalar high);
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
	
	void calculateSlope(squares_t *squares);
};

#endif
