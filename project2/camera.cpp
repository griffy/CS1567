#include "camera.h"
#include <cv.h>
#include <robot_if++.h>

Camera::Camera(RobotInterface *robotInterface) {
    _robotInterface = robotInterface;
    setQuality(RI_CAMERA_QUALITY_HIGH);
    setResolution(RI_CAMERA_RES_320);
}

Camera::~Camera() {

}

void Camera::setQuality(int quality) {
    if (robot->CameraCfg(RI_CAMERA_DEFAULT_BRIGHTNESS, 
                        RI_CAMERA_DEFAULT_CONTRAST, 
                        5, 
                        _resolution, 
                        quality)) {
        LOG.write(LOG_HIGH, "camera settings", 
                  "Failed to change the quality to %d", quality);
    }
    else {
        _quality = quality;
    }
}

void Camera::setResolution(int resolution) {
    if (robot->CameraCfg(RI_CAMERA_DEFAULT_BRIGHTNESS, 
                        RI_CAMERA_DEFAULT_CONTRAST, 
                        5, 
                        resolution, 
                        _quality)) {
        LOG.write(LOG_HIGH, "camera settings", 
                  "Failed to change the resolution to %d", resolution);
    }
    else {
        _resolution = resolution;
    }
}

void Camera::update() {
    _pinkThresholded = getThresholdedImage(RC_PINK_LOW, RC_PINK_HIGH);
    _yellowThresholded = getThresholdedImage(RC_YELLOW_LOW, RC_YELLOW_HIGH);
}

bool Camera::arePinkSquaresInView() {

}

bool Camera::areYellowSquaresInView() {

}

/* Error is defined to be the distance of the square
   from the center of the camera
 */
float Camera::leftSquareDistanceError(int color) {
    switch (color) {
    case COLOR_PINK:
    case COLOR_YELLOW:
    }
}

float Camera::rightSquareDistanceError(int color) {
    switch (color) {
    case COLOR_PINK:
    case COLOR_YELLOW:
    }
}

IplImage* Camera::getHSVImage() {
    // get an image (bgr) from the camera
    IplImage *bgr = getBGRImage();
    if (bgr == NULL) {
        return NULL;
    }

	IplImage *hsv = cvCreateImage(cvGetSize(bgr), IPL_DEPTH_8U, 3);
    // convert the image from BGR to HSV
    cvCvtColor(bgr, hsv, CV_BGR2HSV);
    // free the bgr image
    cvReleaseImage(&bgr);

    return hsv;
}

IplImage* Camera::getThresholdedImage(int low, int high) {
    IplImage *hsv = getHSVImage();
    if (hsv == NULL) {
        return NULL;
    }

    IplImage *thresholded = cvCreateImage(cvGetSize(hsv), IPL_DEPTH_8U, 1);
    // pick out only the color specified by its ranges
    cvInRangeS(hsv, low, high, thresholded);
    // free the hsv image
    cvReleaseImage(&hsv);

    return thresholded;
}

IplImage* Camera::getBGRImage() {
    CvSize size;
    switch (_resolution) {
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
    IplImage *bgr = cvCreateImage(size, IPL_DEPTH_8U, 3);

    if (_robotInterface->getImage(bgr) != RI_RESP_SUCCESS) {
        LOG.write(LOG_HIGH, "camera image", 
                  "Unable to get an image!");
        bgr = NULL;
    }
    return bgr;
}