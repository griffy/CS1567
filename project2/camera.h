#ifndef CS1567_CAMERA_H
#define CS1567_CAMERA_H

#include <opencv/cv.h>
#include <robot_if++.h>

#define COLOR_PINK 0
#define COLOR_YELLOW 1

#define DEFAULT_SQUARE_SIZE 250

class Camera {
public:
	Camera(RobotInterface *robotInterface);
	~Camera();
	void setQuality(int quality);
	void setResolution(int resolution);
	void update();
	int leftSquareDistanceError(int color);
	int rightSquareDistanceError(int color);
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
