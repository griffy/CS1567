#ifndef CS1567_KALMANFILTER_H
#define CS1567_KALMANFILTER_H

extern "C" {
	#include "kalmanfilter/kalmanFilterDef.h"
}
#include "pose.h"

class Kalman {
public:
	Kalman(Pose *initialPose);
	~Kalman();
	void filter(Pose *nsPose, Pose *wePose);
private:
	kalmanFilter _kf;
	float _track[9];

	Pose *_curPose;
};

#endif
