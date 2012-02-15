#include "kalman.h"
#include <stdio.h>

Kalman::Kalman(Pose *initialPose) {
    // make the current pose point to the initial pose
    // such that any modifications made to the value
    // will also update the reference outside this class
	_curPose = initialPose;
    // convert the pose to a 3-element array with x, y, and theta
	float initialPoseArr[3];
	initialPose->toArray(initialPoseArr);
    // create a zero'd velocity array for x, y, and theta
	float velocity[3] = {350/54.0, 0, 0};
    // initialize the kalman filter
	initKalmanFilter(&_kf, initialPoseArr, velocity, 1);
    // initialize the track to zero'd state

	//setUncertainty(.4, .4, .4, .2, .2, .08, .07, .07, .35);

    for (int i = 0; i < 9; i++) {
        _track[0] = 0;
    }
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
	_curPose->setX(_track[0]);
	_curPose->setY(_track[1]);
	_curPose->setTotalTheta(_track[2]);
}

void Kalman::setUncertainty(float proc_x, float proc_y, float proc_theta, float nsx, float nsy, float nstheta, float wex, float wey, float wetheta){
	uncertainties[0]=proc_x;
	uncertainties[1]=proc_y;
	uncertainties[2]=proc_theta;
	uncertainties[3]=nsx;
	uncertainties[4]=nsy;
	uncertainties[5]=nstheta;
	uncertainties[6]=wex;
	uncertainties[7]=wey;
	uncertainties[8]=wetheta;

	rovioKalmanFilterSetUncertainty(&_kf, uncertainties);
}

void Kalman::setProcUncertainty(float x, float y, float theta){
	uncertainties[0]=x;
	uncertainties[1]=y;
	uncertainties[2]=theta;

	rovioKalmanFilterSetUncertainty(&_kf, uncertainties);
}

void Kalman::setNSUncertainty(float x, float y, float theta){
	uncertainties[3]=x;
	uncertainties[4]=y;
	uncertainties[5]=theta;

	rovioKalmanFilterSetUncertainty(&_kf, uncertainties);
}

void Kalman::setWEUncertainty(float x, float y, float theta){
	uncertainties[5]=x;
	uncertainties[6]=y;
	uncertainties[7]=theta;

	rovioKalmanFilterSetUncertainty(&_kf, uncertainties);
}