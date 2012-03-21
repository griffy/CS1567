#include "kalman_filter.h"
#include "constants.h"
#include "logger.h"
#include <stdio.h>

KalmanFilter::KalmanFilter(Pose *initialPose) {
    // make the current pose point to the initial pose
    // such that any modifications made to the value
    // will also update the reference outside this class
	_pose = initialPose;
    // convert the pose to a 3-element array with x, y, and theta
	float initialPoseArr[3];
	initialPose->toArrayForKalman(initialPoseArr);
    // create a zero'd velocity array for x, y, and theta
	_velocity[0] = 0;
	_velocity[1] = 0;
	_velocity[2] = 0;
    // initialize the kalman filter
	initKalmanFilter(&_kf, initialPoseArr, _velocity, 1);
    // initialize the track to zero'd state
    for (int i = 0; i < 9; i++) {
        _track[i] = 0;
    }
    // set the uncertainties to their defaults
    setUncertainty(0.05, 0.05, 0.05,
    			   0.05, 0.05, 0.05,
    			   0.05, 0.05, 0.05);
}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::filter(Pose *nsPose, Pose *wePose) {
    // convert the poses to 3-element arrays, where last
    // is total theta
	float nsPoseArr[3];
	float wePoseArr[3];
	nsPose->toArrayForKalman(nsPoseArr);
	wePose->toArrayForKalman(wePoseArr);

	//Normalize thetas w.r.t each other
	/*
	if(nsPoseArr[2] - wePoseArr[2] > PI) {
		wePoseArr[2] += 2*PI;
	} else if(nsPoseArr[2] - wePoseArr[2] < -PI) {
		wePoseArr[2] -= 2*PI;
	}
	*/
	// just use wheel encoder theta (not total for now)
	wePoseArr[2] = wePose->getTheta();
	nsPoseArr[2] = wePoseArr[2];
    // update the kalman filter with the new data
	rovioKalmanFilter(&_kf, nsPoseArr, wePoseArr, _track);
	LOG.write(LOG_MED, "kalman track", 
              "%f,%f,%f", _track[0], _track[1], _track[2]);
    // update the current pose to its new estimate
    _pose->setX(_track[0]);
	_pose->setY(_track[1]);
	_pose->setTotalTheta(_track[2]);
}

/* Modifies the Kalman velocity estimate */
void KalmanFilter::setVelocity(float x, float y, float theta){
	_velocity[0] = x;
	_velocity[1] = y;
	_velocity[2] = theta;

	rovioKalmanFilterSetVelocity(&_kf, _velocity);
}

/* Sets the uncertainties of the various sensors and overall process */
void KalmanFilter::setUncertainty(float procX, float procY, float procTheta, 
						    float nsX, float nsY, float nsTheta, 
						    float weX, float weY, float weTheta) {
	_uncertainties[0] = procX;
	_uncertainties[1] = procY;
	_uncertainties[2] = procTheta;
	_uncertainties[3] = nsX;
	_uncertainties[4] = nsY;
	_uncertainties[5] = nsTheta;
	_uncertainties[6] = weX;
	_uncertainties[7] = weY;
	_uncertainties[8] = weTheta;

	rovioKalmanFilterSetUncertainty(&_kf, _uncertainties);
}

/* Sets the process uncertainty */
void KalmanFilter::setProcUncertainty(float x, float y, float theta) {
	_uncertainties[0] = x;
	_uncertainties[1] = y;
	_uncertainties[2] = theta;

	rovioKalmanFilterSetUncertainty(&_kf, _uncertainties);
}

/* Sets the north star uncertainty */
void KalmanFilter::setNSUncertainty(float x, float y, float theta) {
	_uncertainties[3] = x;
	_uncertainties[4] = y;
	_uncertainties[5] = theta;

	rovioKalmanFilterSetUncertainty(&_kf, _uncertainties);
}

/* Sets the wheel encoder uncertainty */
void KalmanFilter::setWEUncertainty(float x, float y, float theta) {
	_uncertainties[6] = x;
	_uncertainties[7] = y;
	_uncertainties[8] = theta;

	rovioKalmanFilterSetUncertainty(&_kf, _uncertainties);
}
