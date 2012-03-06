#ifndef CS1567_KALMANFILTER_H
#define CS1567_KALMANFILTER_H

extern "C" {
	#include "lib/kalman/kalmanFilterDef.h"
}
#include "pose.h"

class KalmanFilter {
public:
	KalmanFilter(Pose *initialPose);
	~KalmanFilter();
	void filter(Pose *nsPose, Pose *wePose);
	void setUncertainty(float px, float py, float ptheta,
                        float nsx, float nsy, float nstheta,
                        float wex, float wey, float wetheta);
	void setNSUncertainty(float x, float y, float theta);
	void setWEUncertainty(float x, float y, float theta);
	void setProcUncertainty(float x, float y, float theta);
	void setVelocity(float x, float y, float theta);

	float _velocity[3];
	float _uncertainties[9];
private:
	kalmanFilter _kf;
	float _track[9];

	Pose *_pose;
};

#endif
