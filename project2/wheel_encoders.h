#ifndef CS1567_WHEELENCODERS_H
#define CS1567_WHEELENCODERS_H

#include "position_sensor.h"
#include "fir_filter.h"

class WheelEncoders : public PositionSensor {
public:
	WheelEncoders(RobotInterface *robotInterface);
	~WheelEncoders();
	void updatePose(int room);
private:
	FIRFilter *_filterLeft;
	FIRFilter *_filterRight;
	FIRFilter *_filterRear;

	float _getDeltaX();
	float _getDeltaY();
	float _getDeltaTheta();
	float _getRobotDeltaY();
	float _getFilteredDeltaLeft();
	float _getFilteredDeltaRight();
	float _getFilteredDeltaRear();
};

#endif