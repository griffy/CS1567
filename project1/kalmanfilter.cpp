#include "kalmanfilter.h"

KalmanFilter::KalmanFilter(Pose *initialPose) {
	_curPose = initialPose;
	// TODO: does _kf need to be explicitly initialized?
	float initialPoseArr[3];
	initialPose->toArray(initialPoseArr);
	float velocity[3] = {0, 0, 0};
	initKalmanFilter(&_kf, initialPoseArr, velocity, 1);
}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::filter(Pose *nsPose, Pose *wePose) {
	float nsPoseArr[3];
	float wePoseArr[3];
	nsPose->toArray(nsPoseArr);
	wePose->toArray(wePoseArr);
	rovioKalmanFilter(&_kf, nsPoseArr, wePoseArr, _track);
	_curPose->setX(_track[0]);
	_curPose->setY(_track[1]);
	_curPose->setTheta(_track[2]);
}