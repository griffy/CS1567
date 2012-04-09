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

class Robot; // so we can avoid circular dependency

class PositionSensor {
public:
	PositionSensor(Robot *robot);
	~PositionSensor();
	virtual void updatePose() = 0;
	void resetPose(Pose *pose);
	float getX();
	float getY();
	float getTheta();
	Pose* getPose();
protected:
	Robot *_robot;
	Pose *_pose;
};

#endif