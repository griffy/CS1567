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
	void setUncertainty(float px, float py, float ptheta,float nsx, float nsy, float nstheta,float wex, float wey, float wetheta);
	void setNSUncertainty(float x, float y, float theta);
	void setWEUncertainty(float x, float y, float theta);
	void setProcUncertainty(float x, float y, float theta);
	
	float uncertainties[9];
private:
	kalmanFilter _kf;
	float _track[9];

	Pose *_curPose;
};

#endif
