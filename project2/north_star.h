#ifndef CS1567_NORTHSTAR_H
#define CS1567_NORTHSTAR_H

#include "position_sensor.h"
#include "fir_filter.h"

class NorthStar : public PositionSensor {
public:
	NorthStar(RobotInterface *robotInterface);
	~NorthStar();
	void updatePose(int room);
private:
	FIRFilter *_filterX;
	FIRFilter *_filterY;
	FIRFilter *_filterTheta;
	int _lastRoom;
	std::vector<float> _oldX;
	std::vector<float> _oldY;

	float _getFilteredX();
	float _getFilteredY();
	float _getFilteredTheta();
};

#endif
