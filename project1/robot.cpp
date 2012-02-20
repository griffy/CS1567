#include "robot.h"
#include "phrases.h"
#include <math.h>

#define GOOD_NS_STRENGTH 13222

#define PROC_X_UNCERTAIN 0.05
#define PROC_Y_UNCERTAIN 0.05
#define PROC_THETA_UNCERTAIN 0.1

#define NS_X_UNCERTAIN 0.025
#define NS_Y_UNCERTAIN 0.025
#define NS_THETA_UNCERTAIN 0.1

#define WE_X_UNCERTAIN 0.05
#define WE_Y_UNCERTAIN 0.01
#define WE_THETA_UNCERTAIN 0.025 // was 0.05

Robot::Robot(std::string address, int id) {
    _name = Util::nameFrom(address);
    
    // track when each sensor says we've passed 2PI
    _passed2PIns = false;
    _passed2PIwe = false;
    _numTurns = 0;

    _turnDirection = 0;
    _movingForward = true;
    _speed = 0;

    setFailLimit(MAX_NUM_FAILS);

    _robotInterface = new RobotInterface(address, id);

    printf("robot interface loaded\n");

    _nsXFilter = new FIRFilter("filters/ns_x.ffc");
    _nsYFilter = new FIRFilter("filters/ns_y.ffc");
    _nsThetaFilter = new FIRFilter("filters/ns_theta.ffc"); // empty filter for now
    _weLeftFilter = new FIRFilter("filters/we.ffc");
    _weRightFilter = new FIRFilter("filters/we.ffc");
    _weRearFilter = new FIRFilter("filters/we.ffc");

    printf("fir filters initialized\n");

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
        thetaError = moveToUntil(x, y, MAX_THETA_ERROR);
		float goal = Util::normalizeTheta(_pose->getTheta() + thetaError);
        if (thetaError != 0) {
            turnTo(goal, MAX_THETA_ERROR);
        }
    } while (thetaError != 0);

    _distancePID->flushPID();
    _thetaPID->flushPID();
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

        printf("wheel encoder x: %f\t\ty: %f\t\ttheta: %f\t\ttotal theta: %f\n", _wePose->getX(), 
                                                              _wePose->getY(), _wePose->getTheta(),
                                                              _wePose->getTotalTheta());
        printf("north star x: %f\t\ty: %f\t\ttheta: %f\t\ttotal theta: %f\n", _nsPose->getX(), 
                                                           _nsPose->getY(), _nsPose->getTheta(),
                                                           _nsPose->getTotalTheta(), _nsPose->_numRotations);
        printf("kalman pose x: %f\t\ty: %f\t\ttheta: %f\t\ttotal theta: %f\n", _pose->getX(), 
                                                            _pose->getY(), _pose->getTheta(),
                                                            _pose->getTotalTheta());

        yError = y - _pose->getY();
        xError = x - _pose->getX();
        
        thetaDesired = atan2(yError, xError);
        thetaDesired = Util::normalizeTheta(thetaDesired);

        thetaError = thetaDesired - _pose->getTheta();
        thetaError = Util::normalizeThetaError(thetaError);

        distError = sqrt(yError*yError + xError*xError);

        // TODO: remove
        printf("x error:\t%f\n", xError);
        printf("y error:\t%f\n", yError);
        printf("desired theta: %f\n", thetaDesired);
        printf("theta error:\t%f\n", thetaError);
        printf("distance error:\t%f\n", distError);

        distGain = _distancePID->updatePID(distError);
        _thetaPID->updatePID(thetaError);
		printf("distance gain: %f\n", distGain);

        if (fabs(thetaError) > thetaErrorLimit) {
			printf("theta error of %f too great\n", thetaError);
            return thetaError;
        }

        int moveSpeed = (int)(1.0/distGain);
        printf("moving forward at speed %d\n", moveSpeed);
        moveForward(moveSpeed); // was 4
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

        theta = _pose->getTheta();
        thetaError = thetaGoal - theta;
        thetaError = Util::normalizeThetaError(thetaError);

		printf("theta goal: %f\n", thetaGoal);

        printf("wheel encoder x: %f\t\ty: %f\t\ttheta: %f\t\ttotal theta: %f\n", _wePose->getX(), 
                                                              _wePose->getY(), _wePose->getTheta(),
                                                              _wePose->getTotalTheta());
        printf("north star x: %f\t\ty: %f\t\ttheta: %f\t\ttotal theta: %f\tnumRotations: %d\n", _nsPose->getX(), 
                                                           _nsPose->getY(), _nsPose->getTheta(),
                                                           _nsPose->getTotalTheta(), _nsPose->_numRotations);
        printf("kalman pose x: %f\t\ty: %f\t\ttheta: %f\t\ttotal theta: %f\n", _pose->getX(), 
                                                            _pose->getY(), _pose->getTheta(),
                                                            _pose->getTotalTheta());
        printf("theta error:\t%f\n", thetaError);

        thetaGain = _thetaPID->updatePID(thetaError);

        printf("theta gain: %f\n", thetaGain);

        if (thetaError < -thetaErrorLimit) {
            printf("turning right, since theta error < -limit \n");
            int turnSpeed = (int)(1.0/thetaGain);
            printf("turning at speed %d\n", turnSpeed);
            turnRight(turnSpeed); // was 7
            _numTurns++;
        }
        else if(thetaError > thetaErrorLimit){
            printf("turning left, since theta error > limit\n");
            int turnSpeed = (int)(1.0/thetaGain);
            printf("turning at speed %d\n", turnSpeed);
            turnLeft(turnSpeed); // was 7
            _numTurns++;
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

void Robot::turnLeft(int speed) {
	_turnDirection = 0;
	_movingForward = false;
	_speed = speed;
    _robotInterface->Move(RI_TURN_LEFT, speed);
}

void Robot::turnRight(int speed) {
	_turnDirection = 1;
	_movingForward = false;
	_speed = speed;
    _robotInterface->Move(RI_TURN_RIGHT, speed);
}

void Robot::stop() {
	_movingForward = true;
	_speed = 0;
    _robotInterface->Move(RI_STOP, 0);
}

int Robot::getRoom() {
    return _robotInterface->RoomID();
}

bool Robot::isThereABitchInMyWay() {
    return _robotInterface->IR_Detected();
}

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


    if (getStrength() > GOOD_NS_STRENGTH && _numTurns > 10) { // It's OVER 9000
        //reset the theta on the we
        _wePose->setTheta(_nsPose->getTheta());
        _wePose->setNumRotations(_nsPose->getNumRotations());
        // reset our turn counter, since it's purely for WE uncertainty
        _numTurns = 0;
    }


    if (_speed == 0) {
        _kalmanFilter->setVelocity(0.0, 0.0, 0.0);
    }
    else {
    	if (_movingForward) {
    		float speedX = (SPEED_FORWARD[_speed]);//*cos(_pose->getTheta());
    		float speedY = (SPEED_FORWARD[_speed]);//*sin(_pose->getTheta());
    		printf("predicted speed x and y in cm/s: %f, %f\n", speedX, speedY);
    		_kalmanFilter->setVelocity(speedX, speedY, 0.0);
    	}
    	else {
            float speedTheta = SPEED_TURN[_turnDirection][_speed];
    		printf("predicted speed theta in cm/s: %f\n", speedTheta);
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

// Returns: transformed wheel encoder x estimate in cm of where
//          robot should now be in global coordinate system
float Robot::_getWETransDeltaX() {
    // the robot's x motion should be ignored (since we are not strafing), as that
    // is purely theta information

    // TODO: check logic of this for angles beyond first quadrant
    return Util::weToCM(_getWEDeltaY()) * cos(_wePose->getTheta());
}

// Returns: transformed wheel encoder y estimate in cm of where
//          robot should now be in global coordinate system
float Robot::_getWETransDeltaY() {
    // the robot's x motion should be ignored (since we are not strafing), as that
    // is purely theta information

    // TODO: check logic of this for angles beyond first quadrant
    return Util::weToCM(_getWEDeltaY()) * sin(_wePose->getTheta());
}

// Returns: transformed wheel encoder theta estimate of where
//          robot should now be in global coordinate system
float Robot::_getWETransDeltaTheta() {
    float rearDeltaX = _getWEDeltaXRear();

    // TODO: Should the delta be adjusted according to previous (global) theta?
    return -Util::weToCM(rearDeltaX)/(ROBOT_DIAMETER / 2.0);
}

// Returns: transformed north star x estimate of where
//          robot should now be in global coordinate system
float Robot::_getNSTransX() {
    float result;
    int room = getRoom()-2;
    float coords[2];
    float transform[2];
    float tempTheta;

    coords[0] = _getNSX();
    coords[1] = _getNSY();

    transform[0] = cos(ROOM_ROTATION[room]);
    transform[1] = -sin(ROOM_ROTATION[room]);

    if (room == ROOM_2) {
        tempTheta = .0000204488*coords[0] - .0804;

	    transform[0] = cos(tempTheta);
	    transform[1] = -sin(tempTheta);
    }

    if (ROOM_FLIPX[room]) {
        transform[1] = -transform[1];
        transform[0] = -transform[0];
    }

    Util::mMult(transform, 1, 2, coords, 2, 1, &result);

    //scale
    result /= ROOM_SCALE[room][0];

    //move
    float roomShiftX = COL_OFFSET[0] + ROOM_ORIGINS_FROM_COL[room][0];
    result += roomShiftX;

    return result;
}


// TEMP FUNCTION!
//
// Returns: transformed north star x estimate of where
//          robot should now be in global coordinate system
// TODO?
float Robot::_getNSHalfTransX() {
    float result;
    int room = getRoom();
    float coords[2];
    float transform[2];

    coords[0] = _getNSX();
    coords[1] = _getNSY();
    transform[0] = cos(ROOM_ROTATION[room-2]);
    transform[1] = -sin(ROOM_ROTATION[room-2]);

    if (ROOM_FLIPX[room-2]) {
        transform[1] = -transform[1];
        transform[0] = -transform[0];
    }

    Util::mMult(transform, 1, 2, coords, 2, 1, &result);

    //scale
    //result /= ROOM_SCALE[room][0];

    //move
    float roomShiftX = COL_OFFSET[0] + ROOM_ORIGINS_FROM_COL[room-2][0];
    result += roomShiftX;

    return result;
}


// Returns: transformed north star y estimate of where
// Returns: transformed north star y estimate of where
//          robot should now be in global coordinate system
// TODO?
float Robot::_getNSTransY() {
    float result;
    int room = getRoom()-2;
    float coords[2];
    float transform[2];
    float tempTheta;

    coords[0] = _getNSX();
    coords[1] = _getNSY();

    transform[0] = sin(ROOM_ROTATION[room]);
    transform[1] = cos(ROOM_ROTATION[room]);

    if (room == ROOM_2) {
    	tempTheta = .0000204488*coords[1] - .0804;

	    transform[0] = sin(tempTheta);
	    transform[1] = cos(tempTheta);
    }

    if (ROOM_FLIPY[room]) {
        transform[1] = -transform[1];
        transform[0] = -transform[0];
    }

    Util::mMult(transform, 1, 2, coords, 2, 1, &result);

    //scale
    result /= ROOM_SCALE[room][1];

    //move
    float roomShiftY = COL_OFFSET[1] + ROOM_ORIGINS_FROM_COL[room][1];
    result += roomShiftY;

    //Correction for skew in room 2
    //if (room == ROOM_2) {
    //    result += .75*_getNSTransX();
    //}

    return result;
}


// TEMP FUNCTION!
//
// Returns: transformed north star y estimate of where
//          robot should now be in global coordinate system
// TODO?
float Robot::_getNSHalfTransY() {
    float result;
    int room = getRoom();
    float coords[2];
    float transform[2];

    coords[0] = _getNSX();
    coords[1] = _getNSY();

    transform[0] = sin(ROOM_ROTATION[room-2]);
    transform[1] = cos(ROOM_ROTATION[room-2]);

    if (ROOM_FLIPY[room-2]) {
        transform[1] = -transform[1];
        transform[0] = -transform[0];
    }

    Util::mMult(transform, 1, 2, coords, 2, 1, &result);

    //scale
    //result /= ROOM_SCALE[1][room-2];

    //move
    float roomShiftY = COL_OFFSET[1] + ROOM_ORIGINS_FROM_COL[room-2][1];
    result += roomShiftY;

    return result;
}


// Returns: transformed north star theta estimate of where
//          robot should now be in global coordinate system
// TODO?
float Robot::_getNSTransTheta() {
    float result = _getNSTheta();
    int room = getRoom()-2;
    float tempTheta;
    result -= (ROOM_ROTATION[room]);

/*
    if(room == ROOM_2 && getStrength() < 8000) {
        tempTheta = .0000204488*_getNSX() + 1.4904;
	    result = tempTheta;
    }
*/
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
    
    // account for rotations
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
    
    printf("we lastTheta: %f\n", lastTheta);
    printf("we deltaTheta: %f\n", dTheta);
    printf("we newTheta: %f\n", newTheta);

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

    printf("ns lastTheta: %f\n", lastTheta);
    printf("ns deltaTheta: %f\n", newTheta-lastTheta);
    printf("ns newTheta: %f\n", newTheta);

    _nsPose->setX(newX);
    _nsPose->setY(newY);
    _nsPose->setTheta(newTheta);
}
