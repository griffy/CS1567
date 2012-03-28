/**
 * wheel_encoders.h
 * 
 * @brief 
 * 		This class inherits from the position sensor class.
 *      It performs all functions for the wheel encoders
 *      (except for when to update the data). It keeps a pose, stores 
 *      the raw and filtered wheel encoder data, and contains functions for converting 
 *      the data into global coordinates to be stored back in the pose.
 * 
 * @author
 * 		Shawn Hanna
 * 		Tom Nason
 * 		Joel Griffith
 *
 **/

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