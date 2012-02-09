#include "kalman.h"
#include <stdio.h>

Kalman::Kalman(Pose *initialPose) {
    // make the current pose point to the initial pose
    // such that any modifications made to the value
    // will also update the reference outside this class
	_curPose = initialPose;
	// TODO: does _kf need to be explicitly initialized?
    // convert the pose to a 3-element array with x, y, and theta
	float initialPoseArr[3];
	initialPose->toArray(initialPoseArr);
    // create a zero'd velocity array for x, y, and theta
	float velocity[3] = {0, 0, 0};
    // initialize the kalman filter
	initKalmanFilter(&_kf, initialPoseArr, velocity, 1);
    // initialize the track to zero'd state
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
    printf("%f %f %f\n", nsPoseArr[0], nsPoseArr[1], nsPoseArr[2]);
    printf("%f %f %f\n", wePoseArr[0], wePoseArr[1], wePoseArr[2]);
    // update the kalman filter with the new data
    // FIXME: We have issues here. This causes "Parameter 14 incorrect"
    //        issue
	rovioKalmanFilter(&_kf, nsPoseArr, wePoseArr, _track);
    // update the current pose to its new estimate
	_curPose->setX(_track[0]);
	_curPose->setY(_track[1]);
	_curPose->setTheta(_track[2]);
}
