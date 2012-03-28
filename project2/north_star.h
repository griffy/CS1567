/**
 * north_star.h
 * 
 * @brief 
 * 		This class inherits from the position sensor class.
 *      It performs all functions for the north star sensors 
 *      (except for when to update the data). It keeps a pose, stores 
 *      the raw and filtered north star data, and contains functions for converting 
 *      the data into global coordinates to be stored back in the pose.
 * 
 * @author
 * 		Shawn Hanna
 * 		Tom Nason
 * 		Joel Griffith
 *
 **/

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
