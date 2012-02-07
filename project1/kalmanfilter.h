#ifndef CS1567_KALMANFILTER_H
#define CS1567_KALMANFILTER_H

#include "kalmanFilterDef.h"
#include "pose.h"

class KalmanFilter {
public:
	KalmanFilter(Pose *initialPose);
	~KalmanFilter();
	void filter(Pose *nsPose, Pose *wePose);
private:
	kalmanFilter _kf;
	float _track[9];

	Pose *_curPose;
};

#endif