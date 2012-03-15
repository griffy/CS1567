#ifndef CS1567_CAMERA_H
#define CS1567_CAMERA_H

#define COLOR_PINK 0
#define COLOR_YELLOW 1

class Camera {
public:
	Camera(RobotInterface *robotInterface);
	~Camera();
	void setQuality(int quality);
	void setResolution(int resolution);
	IplImage* getHSVImage();
	IplImage* getBGRImage();
	IplImage* getThresholdedImage();
private:
	RobotInterface *_robotInterface;
	int _quality;
	int _resolution;
	IplImage *_pinkThresholded;
	IplImage *_yellowThresholded;
};

#endif