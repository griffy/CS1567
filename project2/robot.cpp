#include "robot.h"
#include "phrases.h"
#include "logger.h"
#include <math.h>

#define GOOD_NS_STRENGTH 13222

Robot::Robot(std::string address, int id) {
    // store the robot's name as an int to be used later
    _name = Util::nameFrom(address);
    
    // track when each sensor says we've passed 2PI
    _passed2PIns = false;
    _passed2PIwe = false;
    
    //_numTurns = 0;

    // initialize movement variables
    _turnDirection = 0;
    _movingForward = true;
    _speed = 0;

    setFailLimit(MAX_UPDATE_FAILS);

    _robotInterface = new RobotInterface(address, id);

    printf("robot interface loaded\n");

    // initialize filters for sensors
    _nsXFilter = new FIRFilter("filters/ns_x.ffc");
    _nsYFilter = new FIRFilter("filters/ns_y.ffc");
    _nsThetaFilter = new FIRFilter("filters/ns_theta.ffc"); // empty filter for now
    _weLeftFilter = new FIRFilter("filters/we.ffc");
    _weRightFilter = new FIRFilter("filters/we.ffc");
    _weRearFilter = new FIRFilter("filters/we.ffc");

    printf("fir filters initialized\n");

    // initialize robot poses for sensors and kalman
    _wePose = new Pose(0, 0, 0);
    _nsPose = new Pose(0, 0, 0);
    _pose = new Pose(0, 0, 0);

    // bind _pose to the kalman filter
    _kalmanFilter = new Kalman(_pose);

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

    _distancePID = new PID(&distancePIDConstants, MAX_DIST_GAIN, MIN_DIST_GAIN);
    _thetaPID = new PID(&thetaPIDConstants, MAX_THETA_GAIN, MIN_THETA_GAIN);

    printf("pid controllers initialized\n");
}

Robot::~Robot() {
    delete _robotInterface;

    delete _nsXFilter;
    delete _nsYFilter;
    delete _nsThetaFilter;
    delete _weLeftFilter;
    delete _weRightFilter;
    delete _weRearFilter;

    delete _wePose;
    delete _nsPose;
    delete _pose;

    delete _kalmanFilter;

    delete _distancePID;
    delete _thetaPID;
}

void Robot::prefillData() {
    printf("prefilling data...\n");
    for (int i = 0; i < MAX_FILTER_TAPS; i++){
        update();
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
// Moves to a location in the global coordinate system (in cm)
void Robot::moveTo(float x, float y) {
    prefillData();

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

    // reset wheel encoder pose to be north star since we hit our base
    _wePose->setX(_nsPose->getX());
    _wePose->setY(_nsPose->getY());
    _wePose->setTheta(_nsPose->getTheta());
    _wePose->setNumRotations(_nsPose->getNumRotations());
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
        update();

        LOG.write(LOG_LOW, "move_we_pose",
                  "x: %f \t y: %f \t theta: %f \t totalTheta: %f", 
                  _wePose->getX(),
                  _wePose->getY(),
                  _wePose->getTheta(),
                  _wePose->getTotalTheta()); 
        LOG.write(LOG_LOW, "move_ns_pose",
                  "x: %f \t y: %f \t theta: %f \t totalTheta: %f", 
                  _nsPose->getX(),
                  _nsPose->getY(),
                  _nsPose->getTheta(),
                  _nsPose->getTotalTheta()); 
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

    printf("adjusting theta\n");
    do {	
	    update();

        LOG.write(LOG_LOW, "turn_we_pose",
                  "x: %f \t y: %f \t theta: %f \t totalTheta: %f", 
                  _wePose->getX(),
                  _wePose->getY(),
                  _wePose->getTheta(),
                  _wePose->getTotalTheta()); 
        LOG.write(LOG_LOW, "turn_ns_pose",
                  "x: %f \t y: %f \t theta: %f \t totalTheta: %f", 
                  _nsPose->getX(),
                  _nsPose->getY(),
                  _nsPose->getTheta(),
                  _nsPose->getTotalTheta()); 
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
            //_numTurns++;
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
            //_numTurns++;
        }
    } while (fabs(thetaError) > thetaErrorLimit);

    printf("theta acceptable\n");
}

void Robot::moveForward(int speed) {
	_movingForward=true;
	
    if (!isThereABitchInMyWay()) {
		_speed = speed;
		_robotInterface->Move(RI_MOVE_FORWARD, speed);
    }
    else {
		printf("something in the way (ooooOooooOohhh)\n");
		stop();
        printf("turning 90 degrees to compensate...\n");
        turnTo(Util::normalizeTheta(_pose->getTheta()-DEGREE_90),
               MAX_THETA_ERROR);
	}
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

/* Stops the robot, updating movement variables */
void Robot::stop() {
	_movingForward = true;
	_speed = 0;
    _robotInterface->Move(RI_STOP, 0);
}

int Robot::getRoom() {
    return _robotInterface->RoomID();
}

int Robot::getBattery() {
    return _robotInterface->Battery();
}

/* Returns true if something is blocking our robot */
bool Robot::isThereABitchInMyWay() {
    return _robotInterface->IR_Detected();
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

// Updates the robot pose in terms of the global coordinate system
// with the best estimate of its position (using kalman filter)
void Robot::update() {
    // update the robot interface
    _updateInterface();
    // update each pose estimate
    _updateWEPose();
    _updateNSPose();

/*
 * Legacy tuning function
 * 	Knowing that our WE theta value often deviated from real behavior, we opted to trust the NS value after a few turns had been executed,
 * 	under conditions of high signal strength
 *
 *  if (getStrength() > GOOD_NS_STRENGTH && _numTurns > 10) { // It's OVER 9000
 *      //reset the theta on the we
 *      _wePose->setTheta(_nsPose->getTheta());
 *      _wePose->setNumRotations(_nsPose->getNumRotations());
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

            _kalmanFilter->setVelocity(0.0, 0.0, 0.0);
    		//_kalmanFilter->setVelocity(0.0, 0.0, speedTheta);
    	}
    }

    //update the kalman constants
    /*
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
    _kalmanFilter->filter(_nsPose, _wePose);
}

int Robot::getStrength(){
    return _robotInterface->NavStrengthRaw();
}

/* Returns a reference to the Kalman pose */
Pose* Robot::getPose() {
    return _pose;
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

// Returns: filtered wheel encoder (delta) ticks for the left wheel
float Robot::_getWEDeltaLeft() {
    int left = _robotInterface->getWheelEncoder(RI_WHEEL_LEFT);
    return _weLeftFilter->filter((float) left);
}

// Returns: filtered wheel encoder (delta) ticks for the right wheel
float Robot::_getWEDeltaRight() {
    int right = _robotInterface->getWheelEncoder(RI_WHEEL_RIGHT);
    return _weRightFilter->filter((float) right);
}

// Returns: filtered wheel encoder (delta) ticks for the rear wheel
float Robot::_getWEDeltaRear() {
    int rear = _robotInterface->getWheelEncoder(RI_WHEEL_REAR);
    return _weRearFilter->filter((float) rear);
}

// Returns: filtered north star x in ticks
float Robot::_getNSX() {
    int x = _robotInterface->X();
    return _nsXFilter->filter((float) x);
}

// Returns: filtered north star y in ticks
float Robot::_getNSY() {
    int y = _robotInterface->Y();
    return _nsYFilter->filter((float) y);
}

// Returns: filtered north star theta
float Robot::_getNSTheta() {
    float theta = _robotInterface->Theta();
    return _nsThetaFilter->filter(theta);
}

// Returns: filtered wheel encoder delta x for left wheel in ticks
//          in terms of robot axis
float Robot::_getWEDeltaXLeft() {
    float deltaX = _getWEDeltaLeft();
    deltaX *= sin(DEGREE_150);
    return deltaX;
}

// Returns: filtered wheel encoder delta y for left wheel in ticks
//          in terms of robot axis
float Robot::_getWEDeltaYLeft() {
    float deltaY = _getWEDeltaLeft();
    deltaY *= cos(DEGREE_150);
    return -deltaY;
}

// Returns: filtered wheel encoder delta x for right wheel in ticks
//          in terms of robot axis
float Robot::_getWEDeltaXRight() {
    float deltaX = _getWEDeltaRight();
    deltaX *= sin(DEGREE_30);
    return deltaX;
}

// Returns: filtered wheel encoder delta y for right wheel in ticks
//          in terms of robot axis
float Robot::_getWEDeltaYRight() {
    float deltaY = _getWEDeltaRight();
    deltaY *= cos(DEGREE_30);
    return deltaY;
}

// Returns: filtered wheel encoder delta x for rear wheel in ticks
//          in terms of robot axis
float Robot::_getWEDeltaXRear() {
    return _getWEDeltaRear();
}

// Returns: filtered wheel encoder delta y for rear wheel in ticks
//          in terms of robot axis
float Robot::_getWEDeltaYRear() {
    return 0;
}

// Returns: filtered wheel encoder overall delta x in ticks
//          in terms of robot axis
float Robot::_getWEDeltaX() {
    float leftDeltaX = _getWEDeltaXLeft();
    float rightDeltaX = _getWEDeltaXRight();
    float rearDeltaX =_getWEDeltaRear();

    return (leftDeltaX + rightDeltaX + rearDeltaX) / 3.0;
}

// Returns: filtered wheel encoder overall delta y in ticks
//          in terms of robot axis
float Robot::_getWEDeltaY() {
    float leftDeltaY = _getWEDeltaYLeft();
    float rightDeltaY = _getWEDeltaYRight();

    return (leftDeltaY + rightDeltaY) / 2.0;
}

// Returns: transformed wheel encoder delta x estimate in cm of where
//          robot moved in global coordinate system
float Robot::_getWETransDeltaX() {
    // the robot's x motion should be ignored (since we are not strafing), as that
    // is purely theta information
    return Util::weToCM(_getWEDeltaY()) * cos(_wePose->getTheta());
}

// Returns: transformed wheel encoder delta y estimate in cm of where
//          robot moved in global coordinate system
float Robot::_getWETransDeltaY() {
    // the robot's x motion should be ignored (since we are not strafing), as that
    // is purely theta information
    return Util::weToCM(_getWEDeltaY()) * sin(_wePose->getTheta());
}

// Returns: transformed wheel encoder delta theta estimate of angle
//          robot moved at in global coordinate system
float Robot::_getWETransDeltaTheta() {
    float rearDeltaX = _getWEDeltaXRear();

    return -Util::weToCM(rearDeltaX)/(ROBOT_DIAMETER / 2.0);
}

// Returns: transformed north star x estimate of where
//          robot should now be in global coordinate system
//          NorthStar data is: 	
//              - rotated to fit global coordinate system
//				- scaled to centimeters
//				- shifted to global coordinate origin
float Robot::_getNSTransX() {
    float result;
    int room = getRoom()-2; //Room IDs used to index transformation constants
    float coords[2];
    float transform[2];
    float tempTheta;

    //Get NS data
    coords[0] = _getNSX();
    coords[1] = _getNSY();

    //Build rotation matrix
    transform[0] = cos(ROOM_ROTATION[room]);
    transform[1] = -sin(ROOM_ROTATION[room]);

    //Apply specific linear transformation to Room 2, to correct for theta skew
    if (room == ROOM_2) {
        tempTheta = .0000204488*coords[0] - .0804; //Linear fit from observed NS data

	    transform[0] = cos(tempTheta);
	    transform[1] = -sin(tempTheta);
    }

    //Perform matrix multiplication
    Util::mMult(transform, 1, 2, coords, 2, 1, &result);

    //Scale to centimeters
    result /= ROOM_SCALE[room][0];

    //Shift to global coordinate zero 
    float roomShiftX = COL_OFFSET[0] + ROOM_ORIGINS_FROM_COL[room][0];
    result += roomShiftX;

    return result;
}

// Returns: transformed north star y estimate of where
//          robot should now be in global coordinate system
//          NorthStar data is:
//              - rotated to fit global coordinate system
//				- scaled to centimeters
//				- shifted to global coordinate origin
float Robot::_getNSTransY() {
    float result;
    int room = getRoom()-2;
    float coords[2];
    float transform[2];
    float tempTheta;

    //Get NS data
    coords[0] = _getNSX();
    coords[1] = _getNSY();

    //Build rotation matrix
    transform[0] = sin(ROOM_ROTATION[room]);
    transform[1] = cos(ROOM_ROTATION[room]);

    //Apply specific linear transformation to Room 2, to correct for theta skew
    if (room == ROOM_2) {
    	tempTheta = .0000204488*coords[1] - .0804; //Linear fit from observed NS data

	    transform[0] = sin(tempTheta);
	    transform[1] = cos(tempTheta);
    }

    //Perform matrix multiplication
    Util::mMult(transform, 1, 2, coords, 2, 1, &result);

    //Scale to centimeters
    result /= ROOM_SCALE[room][1];

    //Shift to global coordinate zero 
    float roomShiftY = COL_OFFSET[1] + ROOM_ORIGINS_FROM_COL[room][1];
    result += roomShiftY;

    return result;
}



// Returns: transformed north star theta estimate of where
//          robot should now be in global coordinate system
float Robot::_getNSTransTheta() {
    float result = _getNSTheta();
    int room = getRoom()-2;

    //Rotate theta to match offset of NorthStar axes from global coordinate systwm
    result -= (ROOM_ROTATION[room]);

/*  Theta does not require fix in Room 2 coordinate system, so this has been commented out
    if(room == ROOM_2 && getStrength() < 8000) {
        result = .0000204488*_getNSX() + 1.4904;
    }
*/
    //Add additional theta shift for varying definitions of axes between NS and global coords
    result += THETA_SHIFT[room];

    // convert from [-pi, pi] to [0, 2pi]
    result = Util::normalizeTheta(result);
    return result;
}

// Updates transformed wheel encoder pose estimate of where
// robot should now be in global coordinate system
void Robot::_updateWEPose() {
    float lastTheta = _wePose->getTheta();
    
    float deltaX = _getWETransDeltaX();
    float deltaY = _getWETransDeltaY();
    float dTheta = _getWETransDeltaTheta();
   
    float newTheta = Util::normalizeTheta(lastTheta + dTheta);
    
    // Account for rotations
    if ((lastTheta > (3/2.0)*PI && newTheta < PI/2.0) || 
        (lastTheta < PI/2.0 && newTheta > (3/2.0)*PI)) {
        _passed2PIwe = !_passed2PIwe;
    }
    
    if (_passed2PIwe && (lastTheta > PI && newTheta < PI)) {
        _wePose->modifyRotations(1);
        _passed2PIwe = false;
    }
    else if(_passed2PIwe && (lastTheta < PI && newTheta > PI)) {
        _wePose->modifyRotations(-1);
        _passed2PIwe = false;
    }
    
    LOG.write(LOG_MED, "update_we",
              "lastTheta: %f \t deltaTheta: %f \t newTheta: %f",
              lastTheta,
              dTheta,
              newTheta);

    _wePose->setX(_wePose->getX()+deltaX);
    _wePose->setY(_wePose->getY()+deltaY);
    _wePose->setTheta(newTheta);
}
 
// Updates transformed north star pose estimate of where
// robot should now be in global coordinate system
void Robot::_updateNSPose() {
    float lastTheta = _nsPose->getTheta();

    float newX = _getNSTransX();
    float newY = _getNSTransY();
    float newTheta = _getNSTransTheta();
    
    // account for rotations
    if ((lastTheta > (3/2.0)*PI && newTheta < PI/2.0) || 
        (lastTheta < PI/2.0 && newTheta > (3/2.0)*PI)) {
        _passed2PIns = !_passed2PIns;
    }
    
    if (_passed2PIns && (lastTheta > PI && newTheta < PI)) {
        _nsPose->modifyRotations(1);
        _passed2PIns = false;
    }
    else if(_passed2PIns && (lastTheta < PI && newTheta > PI)) {
        _nsPose->modifyRotations(-1);
        _passed2PIns = false;
    }

    LOG.write(LOG_MED, "update_ns",
              "lastTheta: %f \t deltaTheta: %f \t newTheta: %f",
              lastTheta,
              newTheta-lastTheta,
              newTheta);

    _nsPose->setX(newX);
    _nsPose->setY(newY);
    _nsPose->setTheta(newTheta);
}
