#include "kalman.h"
#include <stdio.h>

Kalman::Kalman(Pose *initialPose) {
    // make the current pose point to the initial pose
    // such that any modifications made to the value
    // will also update the reference outside this class
	_pose = initialPose;
    // convert the pose to a 3-element array with x, y, and theta
	float initialPoseArr[3];
	initialPose->toArray(initialPoseArr);
    // create a zero'd velocity array for x, y, and theta
	float velocity[3] = {350/54.0, 0, 0}; // FIXME: should be 0?
    // initialize the kalman filter
	initKalmanFilter(&_kf, initialPoseArr, velocity, 1);
    // initialize the track to zero'd state
    for (int i = 0; i < 9; i++) {
        _track[0] = 0;
    }
    //setUncertainty() // TODO
}

Kalman::~Kalman() {}

void Kalman::filter(Pose *nsPose, Pose *wePose) {
    // convert the poses to 3-element arrays
	float nsPoseArr[3];
	float wePoseArr[3];
	nsPose->toArray(nsPoseArr);
	wePose->toArray(wePoseArr);
    // update the kalman filter with the new data
	rovioKalmanFilter(&_kf, nsPoseArr, wePoseArr, _track);
    // update the current pose to its new estimate
	_pose->setX(_track[0]);
	_pose->setY(_track[1]);
	_pose->setTotalTheta(_track[2]);
}

void Kalman::setUncertainty(float procX, float procY, float procTheta, 
						    float nsX, float nsY, float nsTheta, 
						    float weX, float weY, float weTheta) {
	uncertainties[0] = procX;
	uncertainties[1] = procY;
	uncertainties[2] = procTheta;
	uncertainties[3] = nsX;
	uncertainties[4] = nsY;
	uncertainties[5] = nsTheta;
	uncertainties[6] = weX;
	uncertainties[7] = weY;
	uncertainties[8] = weTheta;

	rovioKalmanFilterSetUncertainty(&_kf, uncertainties);
}

void Kalman::setProcUncertainty(float x, float y, float theta) {
	uncertainties[0] = x;
	uncertainties[1] = y;
	uncertainties[2] = theta;

	rovioKalmanFilterSetUncertainty(&_kf, uncertainties);
}

void Kalman::setNSUncertainty(float x, float y, float theta) {
	uncertainties[3] = x;
	uncertainties[4] = y;
	uncertainties[5] = theta;

	rovioKalmanFilterSetUncertainty(&_kf, uncertainties);
}

void Kalman::setWEUncertainty(float x, float y, float theta) {
	uncertainties[6] = x;
	uncertainties[7] = y;
	uncertainties[8] = theta;

	rovioKalmanFilterSetUncertainty(&_kf, uncertainties);
}