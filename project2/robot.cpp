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

    _distancePID = new PID(&distancePIDConstants, MAX_DIST_GAIN, MIN_DIST_GAIN);
    _thetaPID = new PID(&thetaPIDConstants, MAX_THETA_GAIN, MIN_THETA_GAIN);
    _centerPID = new PID(&centerPIDConstants, MAX_CENTER_GAIN, MIN_CENTER_GAIN);

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
        moveTo(goalX, goalY);

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
        // cap our speed at 6, since going too slow causes problems
        if (moveSpeed > 6) {
            moveSpeed = 6;
        }

        LOG.write(LOG_MED, "pid_speeds", "forward: %d", moveSpeed);

        moveForward(moveSpeed);
    } while (distError > MAX_DIST_ERROR);

    return 0; // no error when we've finished
}

void Robot::turnTo(float thetaGoal, float thetaErrorLimit) {
    float theta;
    float thetaError;

    float thetaGain;
 
    //REMOVE THIS
    thetaGoal = 0.0;

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
            // cap our speed at 6, since turning too slow causes problems
            if (turnSpeed > 6) {
                turnSpeed = 6;
            }

            LOG.write(LOG_MED, "pid_speeds", "turn: %d", turnSpeed);

            turnRight(turnSpeed);
        }
        else if(thetaError > thetaErrorLimit){
            LOG.write(LOG_MED, "turn_adjust", 
                      "direction: left, since theta error > limit");
            int turnSpeed = (int)fabs((1.0/thetaGain));
            if (turnSpeed > 6) {
                turnSpeed = 6;
            }

            LOG.write(LOG_MED, "pid_speeds", "turn: %d", turnSpeed);

            turnLeft(turnSpeed);
        }
    } while (fabs(thetaError) > thetaErrorLimit);

    printf("theta acceptable\n");
}

void Robot::center() {
    while (true) {
        // TODO: if this is unreliable, try filtering
        // the errors and taking the filtered average to work with
        updateCamera();

        float centerError = _camera->centerError(COLOR_PINK);
        float centerGain = _centerPID->updatePID(centerError);

        LOG.write(LOG_LOW, "center", "center error: %f", centerError);
        LOG.write(LOG_LOW, "center", "center gain: %f", centerGain);

        if (fabs(centerError) < MAX_CENTER_ERROR) {
            // we're close enough to centered, so stop adjusting
            LOG.write(LOG_LOW, "center", "Center error: %f < %f, stop correcting.", centerError, MAX_CENTER_ERROR);
            break;
        }

        int strafeSpeed = (int)fabs((1.0/centerGain));
        // cap our speed at 4, since moving too slow is bad
        if (strafeSpeed > 4) {
            strafeSpeed = 4;
        }
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
    _robotInterface->Move(RI_MOVE_LEFT, speed);
}

/* Strafes the robot right at the given speed */
void Robot::strafeRight(int speed) {
    _robotInterface->Move(RI_MOVE_RIGHT, speed);
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

/*
 * Legacy tuning function
 * 	Knowing that our WE theta value often deviated from real behavior, we opted to trust the NS value after a few turns had been executed,
 * 	under conditions of high signal strength
 *
 *  if (getStrength() > GOOD_NS_STRENGTH && _numTurns > 10) { // It's OVER 9000
 *      //reset the theta on the we
 *      _wheelEncoders->getPose()->setTheta(_northStar->getPose()->getTheta());
 *      _wheelEncoders->getPose()->setNumRotations(_northStar->getPose()->getNumRotations());
 *      // reset our turn counter, since it's purely for WE uncertainty
 *      _numTurns = 0;
 *  }
 */

    if (_speed == 0) {
        _kalmanFilter->setVelocity(0.0, 0.0, 0.0);
    }
    else {
    	if (_movingForward) {
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

    /*
    //update the kalman constants
    float newX = 1000.0/getStrength(); 
    float newY = 1500.0/getStrength();
    float newTheta = 800.0/getStrength();
    if (newX > 0.3) {
        newX = .3;
    }
    if (newY > 0.3) {
        newY = .3;
    }
    if (newTheta > 0.3) {
        newTheta = .3;
    }
    */

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
  
    //Legacy tuning function, attempting to dynamically modify trust of the robot's state due to known deviations in WE theta from real behavior
    // the more we turn, the less reliable wheel encoders become
    /*
    float weTurnUncertainty = (_numTurns / 5) * 0.01;
    if (weTurnUncertainty > 0.1) {
        weTurnUncertainty = 0.1;
    }
    _kalmanFilter->setWEUncertainty(WE_X_UNCERTAIN+weTurnUncertainty,
                                    WE_Y_UNCERTAIN+weTurnUncertainty,
                                    WE_THETA_UNCERTAIN+(weTurnUncertainty*2.0));
    */

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
