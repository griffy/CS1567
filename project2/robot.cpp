/**
 * robot.cpp
 * 
 * @brief 
 * 		This class defines the rovio robot, and performs operations that the system can do, such as movement and 
 * 		requesting sensor updates.  It also stores the sensor classes and vehicle position
 * 
 * @author
 * 		Tom Nason
 * 		Joel Griffith
 * 		Shawn Hanna
 * 
 * @date
 * 		created - 2/2/2012
 * 		modified - 3/24/2012
 **/

#include "robot.h"
#include "phrases.h"
#include "logger.h"
#include <math.h>

#define GOOD_NS_STRENGTH 13222

#define MAX_CAMERA_BRIGHTNESS (0x7F)
#define CAMERA_FRAMERATE 5

Robot::Robot(std::string address, int id) {
    // store the robot's name as an int to be used later
    _name = Util::nameFrom(address);
    
    // initialize movement variables
    _turnDirection = 0;
    _movingForward = true;
    _speed = 0;

    setFailLimit(MAX_UPDATE_FAILS);

    _robotInterface = new RobotInterface(address, id);

    printf("robot interface loaded\n");

    // initialize camera
    _camera = new Camera(_robotInterface);

    // initialize position sensors
    _wheelEncoders = new WheelEncoders(_robotInterface);
    _northStar = new NorthStar(_robotInterface);
    
    // initialize global pose
    _pose = new Pose(0, 0, 0);
    // bind _pose to the kalman filter
    _kalmanFilter = new KalmanFilter(_pose);
    _kalmanFilter->setUncertainty(PROC_X_UNCERTAIN,
                                  PROC_Y_UNCERTAIN,
                                  PROC_THETA_UNCERTAIN,
                                  NS_X_UNCERTAIN,
                                  NS_Y_UNCERTAIN,
                                  NS_THETA_UNCERTAIN,
                                  WE_X_UNCERTAIN,
                                  WE_Y_UNCERTAIN,
                                  WE_THETA_UNCERTAIN);

    printf("kalman filter initialized\n");

    PIDConstants distancePIDConstants = {PID_DIST_KP, PID_DIST_KI, PID_DIST_KD};
    PIDConstants thetaPIDConstants = {PID_THETA_KP, PID_THETA_KI, PID_THETA_KD};
    PIDConstants centerPIDConstants = {PID_CENTER_KP, PID_CENTER_KI, PID_CENTER_KD};
    PIDConstants turnCenterPIDConstants = {PID_TURN_CENTER_KP, PID_TURN_CENTER_KI, PID_TURN_CENTER_KD};

    _distancePID = new PID(&distancePIDConstants, MAX_DIST_GAIN, MIN_DIST_GAIN);
    _thetaPID = new PID(&thetaPIDConstants, MAX_THETA_GAIN, MIN_THETA_GAIN);
    _centerPID = new PID(&centerPIDConstants, MAX_CENTER_GAIN, MIN_CENTER_GAIN);
    _turnCenterPID = new PID(&turnCenterPIDConstants, MAX_TURN_CENTER_GAIN, MIN_TURN_CENTER_GAIN);

    printf("pid controllers initialized\n");
    
    prefillData();
    // base the wheel encoder pose off north star to start (since we
    // might start anywhere in the global system)
    //_wheelEncoders->getPose()->setX(_northStar->getX());
    //_wheelEncoders->getPose()->setY(_northStar->getY());
    _wheelEncoders->resetPose(_northStar->getPose());
    // now update our pose again so our global pose isn't
    // some funky value
    updatePose();
}

Robot::~Robot() {
    delete _robotInterface;

    delete _camera;

    delete _wheelEncoders;
    delete _northStar;

    delete _pose;

    delete _kalmanFilter;

    delete _distancePID;
    delete _thetaPID;
    delete _centerPID;
}

/* Returns a reference to the Kalman pose */
Pose* Robot::getPose() {
    return _pose;
}

void Robot::prefillData() {
    printf("prefilling data...\n");
    for (int i = 0; i < MAX_FILTER_TAPS; i++){
        updatePose();
    }
    printf("sufficient data collected\n");
}

void Robot::printBeginPhrase() {
    printf(BEGIN_PHRASES[_name].c_str());
    printf("\n");
}

void Robot::printSuccessPhrase() {
    printf(SUCCESS_PHRASES[_name].c_str());
    printf("\n");
}

void Robot::printFailPhrase() {
    printf(FAIL_PHRASES[_name].c_str());
    printf("\n");
}

/* Celebrates by moving head up and down */
void Robot::rockOut() {
    for (int i = 0; i < 1; i++) {
        _robotInterface->Move(RI_HEAD_UP, 1);
        sleep(1);
        _robotInterface->Move(RI_HEAD_DOWN, 1);
        sleep(1);
    }
}

void Robot::move(int direction, int numCells) {
    int cellsTraveled = 0;

    while (cellsTraveled < numCells) {
        // first attempt to center ourselves before moving (except not first)
        center();
        // reset the wheel encoder totals
        _robotInterface->reset_state();
		updatePose();
        // based on the direction, move in the global coord system
        float goalX = _pose->getX();
        float goalY = _pose->getY();
        switch (direction) {
        case DIR_NORTH:
            goalY += CELL_SIZE;
            break;
        case DIR_SOUTH:
            goalY -= CELL_SIZE;
            break;
        case DIR_EAST:
            goalX += CELL_SIZE;
            break;
        case DIR_WEST:
            goalX -= CELL_SIZE;
            break;
        }
        moveToCell(goalX, goalY);
        cellsTraveled++;
        LOG.write(LOG_LOW, "move", "MADE IT TO CELL %d\n\n\n", cellsTraveled);
    }
}

void Robot::turn(int direction, float radians) {
    float goalTheta;

    if (direction == DIR_LEFT) {
        goalTheta = Util::normalizeTheta(_pose->getTheta()+radians);
        turnTo(goalTheta, MAX_THETA_ERROR);
    }
    else {
        goalTheta = Util::normalizeTheta(_pose->getTheta()-radians);
        turnTo(goalTheta, MAX_THETA_ERROR);
    }
}

// Moves to a cell in the global coord system at the specified cm,
// using both kalman and square detection for centering as it moves
void Robot::moveToCell(float x, float y) {
    printf("beginning move to cell at (%f, %f)\n", x, y);
	printf("Current location: %f, %f, %f", _pose->getX(), _pose->getY(), _pose->getTheta());
    float thetaError;
    do {
        // move to the location until theta is off by too much
        thetaError = moveToUntil(x, y, MAX_THETA_ERROR);
        float goal = Util::normalizeTheta(_pose->getTheta() + thetaError);
        if (thetaError != 0) {
            // if we're off in theta, turn to adjust 
            turnTo(goal, MAX_THETA_ERROR);
            // finally, center between squares as a sanity check
            turnCenter();
        }
    } while (thetaError != 0);

    _distancePID->flushPID();
    _thetaPID->flushPID();

    // reset wheel encoder pose to be Kalman pose since we hit our base
    _wheelEncoders->resetPose(_pose);
}

// Moves to a location in the global coordinate system (in cm)
void Robot::moveTo(float x, float y) {
    printf("beginning move\n");

    float thetaError;
    do {
        // move to the location until theta is off by too much
        thetaError = moveToUntil(x, y, MAX_THETA_ERROR);
		float goal = Util::normalizeTheta(_pose->getTheta() + thetaError);
        if (thetaError != 0) {
            // if we're off in theta, turn to adjust 
            turnTo(goal, MAX_THETA_ERROR);
        }
    } while (thetaError != 0);

    _distancePID->flushPID();
    _thetaPID->flushPID();

    // reset wheel encoder pose to be Kalman pose since we hit our base
    _wheelEncoders->resetPose(_pose);
}

// Moves to a location in the global coordinate system (in cm) 
// until theta error limit exceeded
// Returns: theta error
float Robot::moveToUntil(float x, float y, float thetaErrorLimit) {
    float yError;
    float xError;
    float thetaDesired;
    float thetaError;
    float distError;

    float distGain;

    printf("heading toward (%f, %f)\n", x, y);
    do {
        updatePose();

        LOG.write(LOG_LOW, "move_we_pose",
                  "x: %f \t y: %f \t theta: %f \t totalTheta: %f", 
                  _wheelEncoders->getPose()->getX(),
                  _wheelEncoders->getPose()->getY(),
                  _wheelEncoders->getPose()->getTheta(),
                  _wheelEncoders->getPose()->getTotalTheta());
        LOG.write(LOG_LOW, "move_ns_pose",
                  "x: %f \t y: %f \t theta: %f \t totalTheta: %f", 
                  _northStar->getPose()->getX(),
                  _northStar->getPose()->getY(),
                  _northStar->getPose()->getTheta(),
                  _northStar->getPose()->getTotalTheta()); 
        LOG.write(LOG_LOW, "move_kalman_pose",
                  "x: %f \t y: %f \t theta: %f \t totalTheta: %f", 
                  _pose->getX(),
                  _pose->getY(),
                  _pose->getTheta(),
                  _pose->getTotalTheta()); 

        yError = y - _pose->getY();
        xError = x - _pose->getX();

        thetaDesired = atan2(yError, xError);
        thetaDesired = Util::normalizeTheta(thetaDesired);

        thetaError = thetaDesired - _pose->getTheta();
        thetaError = Util::normalizeThetaError(thetaError);

        distError = sqrt(yError*yError + xError*xError);

        LOG.write(LOG_MED, "move_error_estimates",
                  "x err: %f \t y err: %f \t distance err: %f \t "
                  "theta err: %f \t theta desired: %f \t", 
                  xError,
                  yError,
                  distError,
                  thetaError,
                  thetaDesired);

        distGain = _distancePID->updatePID(distError);
        _thetaPID->updatePID(thetaError);

        LOG.write(LOG_LOW, "move_gain", "dist: %f", distGain);

        if (fabs(thetaError) > thetaErrorLimit) {
			printf("theta error of %f too great\n", thetaError);
            return thetaError;
        }
        int moveSpeed = (int)(1.0/distGain);
        moveSpeed = Util::capSpeed(moveSpeed, 6);

        LOG.write(LOG_MED, "pid_speeds", "forward: %d", moveSpeed);

        moveForward(moveSpeed);
    } while (distError > MAX_DIST_ERROR);

    return 0; // no error when we've finished
}

void Robot::turnTo(float thetaGoal, float thetaErrorLimit) {
    float theta;
    float thetaError;

    float thetaGain;
 
    printf("adjusting theta\n");
    do {	
	    updatePose();

        LOG.write(LOG_LOW, "turn_we_pose",
                  "x: %f \t y: %f \t theta: %f \t totalTheta: %f", 
                  _wheelEncoders->getPose()->getX(),
                  _wheelEncoders->getPose()->getY(),
                  _wheelEncoders->getPose()->getTheta(),
                  _wheelEncoders->getPose()->getTotalTheta()); 
        LOG.write(LOG_LOW, "turn_ns_pose",
                  "x: %f \t y: %f \t theta: %f \t totalTheta: %f", 
                  _northStar->getPose()->getX(),
                  _northStar->getPose()->getY(),
                  _northStar->getPose()->getTheta(),
                  _northStar->getPose()->getTotalTheta()); 
        LOG.write(LOG_LOW, "turn_kalman_pose",
                  "x: %f \t y: %f \t theta: %f \t totalTheta: %f", 
                  _pose->getX(),
                  _pose->getY(),
                  _pose->getTheta(),
                  _pose->getTotalTheta()); 

        theta = _pose->getTheta();
        thetaError = thetaGoal - theta;
        thetaError = Util::normalizeThetaError(thetaError);

        LOG.write(LOG_MED, "turn_error_estimates",
                  "theta err: %f \t theta goal: %f",
                  thetaError,
                  thetaGoal);

        thetaGain = _thetaPID->updatePID(thetaError);

        LOG.write(LOG_LOW, "turn_gain", "theta: %f", thetaGain);

        if (thetaError < -thetaErrorLimit) {
            LOG.write(LOG_MED, "turn_adjust", 
                      "direction: right, since theta error < -limit");
            int turnSpeed = (int)fabs((1.0/thetaGain));
            turnSpeed = Util::capSpeed(turnSpeed, 6);

            LOG.write(LOG_MED, "pid_speeds", "turn: %d", turnSpeed);

            turnRight(turnSpeed);
        }
        else if(thetaError > thetaErrorLimit){
            LOG.write(LOG_MED, "turn_adjust", 
                      "direction: left, since theta error > limit");
            int turnSpeed = (int)fabs((1.0/thetaGain));
            turnSpeed = Util::capSpeed(turnSpeed, 6);

            LOG.write(LOG_MED, "pid_speeds", "turn: %d", turnSpeed);

            turnLeft(turnSpeed);
        }
    } while (fabs(thetaError) > thetaErrorLimit);

    printf("theta acceptable\n");
}

void Robot::turnCenter() {
    while (true) {
        sleep(1);

        updateCamera();

        float turnCenterError = _camera->centerError(COLOR_PINK);
        float turnCenterGain = _turnCenterPID->updatePID(turnCenterError);

        LOG.write(LOG_LOW, "turnCenter", "turn center error: %f", turnCenterError);
        LOG.write(LOG_LOW, "turnCenter", "turn center gain: %f", turnCenterGain);

        if (fabs(turnCenterError) < MAX_TURN_CENTER_ERROR) {
            // we're close enough to centered, so stop adjusting
            LOG.write(LOG_LOW, "turnCenter", 
                      "turn center error: |%f| < %f, stop correcting.", 
                      turnCenterError, 
                      MAX_TURN_CENTER_ERROR);
            break;
        }

        int turnSpeed = (int)fabs((1.0/turnCenterGain));
        turnSpeed = Util::capSpeed(turnSpeed, 6);

        LOG.write(LOG_LOW, "pid_speeds", "turn: %d", turnSpeed);

        if (turnCenterError < 0) {
            LOG.write(LOG_LOW, "turnCenter", "Turn center error: %f, move right", turnCenterError);
            turnRight(turnSpeed);
        }
        else {
            LOG.write(LOG_LOW, "turnCenter", "Turn center error: %f, move left", turnCenterError);
            turnLeft(turnSpeed);
        }
    }

    _turnCenterPID->flushPID();
}

void Robot::center() {
    while (true) {
        updateCamera();

        float centerError = _camera->centerError(COLOR_PINK);
        float centerGain = _centerPID->updatePID(centerError);
		
		if(centerError > 5){
			if(centerError < 100){
				//probably looking at left side, turn right
				LOG.write(LOG_HIGH, "center", "predict wall error -> turn right");
				turnLeft(3);
			}
			else if(centerError > 100){
				LOG.write(LOG_HIGH, "center", "predict wall error -> turn left");
				turnRight(3);
			}
			else{
				LOG.write(LOG_HIGH, "center", "predicted a wall error -> don't know which one");
			}
		}
		else{
			LOG.write(LOG_LOW, "center", "\t\t\t\tcenter error: %f", centerError);
			/// TODO remove break
			//break;
			LOG.write(LOG_LOW, "center", "center gain: %f", centerGain);

			if (fabs(centerError) < MAX_CENTER_ERROR) {
				// we're close enough to centered, so stop adjusting
				LOG.write(LOG_LOW, "center", "Center error: |%f| < %f, stop correcting.", centerError, MAX_CENTER_ERROR);
				break;
			}

			int strafeSpeed = (int)fabs((centerGain));
            strafeSpeed = Util::capSpeed(strafeSpeed, 8);
			
			LOG.write(LOG_LOW, "pid_speeds", "strafe: %d", strafeSpeed);

			if (centerError < 0) {
				LOG.write(LOG_LOW, "center", "Center error: %f, move right", centerError);
				strafeRight(strafeSpeed);
			}
			else {
				LOG.write(LOG_LOW, "center", "Center error: %f, move left", centerError);
				strafeLeft(strafeSpeed);
			}
		}
    }

    _centerPID->flushPID();
}

void Robot::moveForward(int speed) {
	_movingForward=true;
    _speed = speed;
    _robotInterface->Move(RI_MOVE_FORWARD, speed);
}

/* Turns the robot left at the given speed, updating movement variables */
void Robot::turnLeft(int speed) {
	_turnDirection = 0;
	_movingForward = false;
	_speed = speed;
    _robotInterface->Move(RI_TURN_LEFT, speed);
}

/* Turns the robot right at the given speed, updating movement variables */
void Robot::turnRight(int speed) {
	_turnDirection = 1;
	_movingForward = false;
	_speed = speed;
    _robotInterface->Move(RI_TURN_RIGHT, speed);
}

/* Strafes the robot left at the given speed */
void Robot::strafeLeft(int speed) {
	_speed=0;
    // we need about 5 commands to actually strafe sideways
    for (int i = 0; i < 5; i++) {
        _robotInterface->Move(RI_MOVE_LEFT, 10);
    }
}

/* Strafes the robot right at the given speed */
void Robot::strafeRight(int speed) {
	_speed=0;
    for (int i = 0; i < 5; i++) {
        _robotInterface->Move(RI_MOVE_RIGHT, 10);
    }
}

/* Stops the robot, updating movement variables */
void Robot::stop() {
	_movingForward = true;
	_speed = 0;
    _robotInterface->Move(RI_STOP, 0);
}

int Robot::getRoom() {
    return _robotInterface->RoomID() - 2;
}

int Robot::getBattery() {
    return _robotInterface->Battery();
}

int Robot::getStrength(){
    return _robotInterface->NavStrengthRaw();
}

/* Returns true if something is blocking our robot */
bool Robot::isThereABitchInMyWay() {
    return _robotInterface->IR_Detected();
}

void Robot::updateCamera() {
    _camera->update();
}

// Updates the robot pose in terms of the global coordinate system
// with the best estimate of its position (using kalman filter)
void Robot::updatePose() {
    // update the robot interface
    _updateInterface();
    // update each pose estimate
    _northStar->updatePose(getRoom());
    _wheelEncoders->updatePose(getRoom());

    if (_speed <= 0) {
		_speed=0;
        _kalmanFilter->setVelocity(0.0, 0.0, 0.0);
    }
    else {
    	if (_movingForward) {
			printf("Speed: %d\n", _speed);
    		float speedX = SPEED_FORWARD[_speed];
    		float speedY = SPEED_FORWARD[_speed];

            LOG.write(LOG_MED, "update_predictions", 
                      "speed x (cm/s): %f \t speed y (cm/s): %f",
                      speedX,
                      speedY);

    		_kalmanFilter->setVelocity(speedX, speedY, 0.0);
    	}
    	else {
            float speedTheta = SPEED_TURN[_turnDirection][_speed]; //Fetch turning speed in radians per second

            LOG.write(LOG_MED, "update_predictions", 
                      "speed theta (cm/s): %f", speedTheta);

            //_kalmanFilter->setVelocity(0.0, 0.0, 0.0);
    		_kalmanFilter->setVelocity(0.0, 0.0, speedTheta);
    	}
    }

    // if we're in room 2, don't trust north star so much
    if (getRoom() == ROOM_2) {
        _kalmanFilter->setNSUncertainty(NS_X_UNCERTAIN+0.05, 
                                        NS_Y_UNCERTAIN+0.15, 
                                        NS_THETA_UNCERTAIN+0.05);
    } 
    else {
        _kalmanFilter->setNSUncertainty(NS_X_UNCERTAIN,
                                        NS_Y_UNCERTAIN,
                                        NS_THETA_UNCERTAIN);
    }

    // pass updated poses to kalman filter and update main pose
    _kalmanFilter->filter(_northStar->getPose(), 
                          _wheelEncoders->getPose());
}

// Attempts to update the robot
// Returns: true if update succeeded
//          false if update fail limit was reached
bool Robot::_updateInterface() {
    int failCount = 0;
    int failLimit = getFailLimit();

    while (_robotInterface->update() != RI_RESP_SUCCESS &&
           failCount < failLimit) {
        failCount++;
    }

    if (failCount >= failLimit) {
        return false;
    }
    return true;
}

/* Sets the amount of times an attempt to update the robot's
 * interface can fail before we give up
 */
void Robot::setFailLimit(int limit) {
    _failLimit = limit;
}

int Robot::getFailLimit() {
    return _failLimit;
}

void Robot::setCameraResolution(int resolution, int quality) {
    _camera->setResolution(resolution);
    _camera->setQuality(quality);
}
