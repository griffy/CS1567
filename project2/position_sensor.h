/**
 * position_sensor.h
 * 
 * @brief 
 * 		This is an abstract class intended to be inherited from
 *      by a sensor class that keeps a global pose.
 * 
 * @author
 * 		Shawn Hanna
 * 		Tom Nason
 * 		Joel Griffith
 *
 **/

#ifndef CS1567_POSITIONSENSOR_H
#define CS1567_POSITIONSENSOR_H

#include "pose.h"
#include <robot_if++.h>

class PositionSensor {
public:
	PositionSensor(RobotInterface *robotInterface);
	~PositionSensor();
	virtual void updatePose(int room) = 0;
	void resetPose(Pose *pose);
	float getX();
	float getY();
	float getTheta();
	Pose* getPose();
protected:
	RobotInterface *_robotInterface;
	Pose *_pose;
};

#endif