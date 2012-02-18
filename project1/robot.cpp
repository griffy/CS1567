#include "robot.h"
#include <math.h>

Robot::Robot(std::string address, int id) {
    _robotInterface = new RobotInterface(address, id);

    _nsXFilter = new FIRFilter("filters/ns_x.ffc");
    _nsYFilter = new FIRFilter("filters/ns_y.ffc");
    _nsThetaFilter = new FIRFilter("filters/ns_theta.ffc"); // empty filter for now
    _weLeftFilter = new FIRFilter("filters/we.ffc");
    _weRightFilter = new FIRFilter("filters/we.ffc");
    _weRearFilter = new FIRFilter("filters/we.ffc");

    name = address;
    
    _passed2PIns = false;
    _passed2PIwe = false;

    setFailLimit(MAX_NUM_FAILS);

    _wePose = new Pose(0, 0, 0);
    _nsPose = new Pose(0, 0, 0);
    _pose = new Pose(0, 0, 0);

    // bind _pose to the kalman filter
    _kalmanFilter = new Kalman(_pose);

    printf("kf initialized\n");

    PIDConstants distancePIDConstants = {PID_DIST_KP, PID_DIST_KI, PID_DIST_KD};
    PIDConstants thetaPIDConstants = {PID_THETA_KP, PID_THETA_KI, PID_THETA_KD};

    _distancePID = new PID(&distancePIDConstants, MAX_DIST_GAIN, MIN_DIST_GAIN);
    _thetaPID = new PID(&thetaPIDConstants, MAX_THETA_GAIN, MIN_THETA_GAIN);

    printf("pid controllers initialized\n");

	// forward time constants
	_forwardSpeed[0] = 0.0;
	_forwardSpeed[1] = 3.5;
	_forwardSpeed[2] = 3.5;
	_forwardSpeed[3] = 3.5;
	_forwardSpeed[4] = 3.7;
	_forwardSpeed[5] = 3.7;
	_forwardSpeed[6] = 3.9;
	_forwardSpeed[7] = 4.6;
	_forwardSpeed[8] = 4.6;
	_forwardSpeed[9] = 4.8;
	_forwardSpeed[10] = 4.9;

	// left time constants
	_turnSpeed[0][0] = 0.0;
	_turnSpeed[0][1] = 1.8;
	_turnSpeed[0][2] = 1.9;
	_turnSpeed[0][3] = 2.23;
	_turnSpeed[0][4] = 2.36;
	_turnSpeed[0][5] = 2.87;
	_turnSpeed[0][6] = 2.8;
	_turnSpeed[0][7] = 4.6;
	_turnSpeed[0][8] = 4.75;
	_turnSpeed[0][9] = 5.35;
	_turnSpeed[0][10] = 5.3;

	//right time constants
	_turnSpeed[1][0] = 0.0;
	_turnSpeed[1][1] = 1.6;
	_turnSpeed[1][2] = 2.0;
	_turnSpeed[1][3] = 2.2;
	_turnSpeed[1][4] = 2.3;
	_turnSpeed[1][5] = 2.8;
	_turnSpeed[1][6] = 2.8;
	_turnSpeed[1][7] = 4.75;
	_turnSpeed[1][8] = 4.75;
	_turnSpeed[1][9] = 5.35;
	_turnSpeed[1][10] = 5.45;

	_turnDirection = 0;
	_movingForward = true;
	_speedDistance = 116.0; // cm
	_speed = 0;

    prefillData();
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

    delete _distancePID;
    delete _thetaPID;
}

void Robot::prefillData() {
    printf("Prefilling data...\n");
    for (int i = 0; i < MAX_FILTER_TAPS; i++){
        update();
    }
}

// Moves to a location in the global coordinate system (in cm)
void Robot::moveTo(float x, float y) {
    float thetaError;

    _robotInterface->Move(RI_HEAD_MIDDLE, 1);

    printf("prefilling data for move...\n");
    for (int i = 0; i < MAX_FILTER_TAPS; i++) {
        update();
    }
    printf("beginning move\n");
    do {
		printf("We are in room: %d\n", getRoom());
        thetaError = moveToUntil(x, y, MAX_THETA_ERROR);
		float goal = Util::normalizeTheta(_pose->getTheta() + thetaError);
		printf("Finished MoveTo ==> goal=%f\n", goal);
        if (thetaError != 0) {
            turnTo(goal, MAX_THETA_ERROR);
        }
    } while (thetaError != 0);

    _distancePID->flushPID();
    _thetaPID->flushPID();

    _robotInterface->Move(RI_HEAD_DOWN, 1);
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
                                                           _nsPose->getTotalTheta());
        printf("kalman pose x: %f\t\ty: %f\t\ttheta: %f\t\ttotal theta: %f\n", _pose->getX(), 
                                                            _pose->getY(), _pose->getTheta(),
                                                            _pose->getTotalTheta());

        yError = y - _pose->getY();
        xError = x - _pose->getX();
        
        thetaDesired = atan2(yError, xError);
        thetaDesired = Util::normalizeTheta(thetaDesired);

        printf("desired theta: %f\n", thetaDesired);

        thetaError = thetaDesired - _pose->getTheta();
        thetaError = Util::normalizeThetaError(thetaError);

        distError = sqrt(yError*yError + xError*xError);

        // TODO: remove
        printf("x error:\t%f\n", xError);
        printf("y error:\t%f\n", yError);
        printf("theta error:\t%f\n", thetaError);
        printf("distance error:\t%f\n", distError);

        distGain = _distancePID->updatePID(distError);
        _thetaPID->updatePID(thetaError);

        if (fabs(thetaError) > thetaErrorLimit/* && xError < MAX_DIST_ERROR+20*/) {
			printf("Returning with error: %f\n", thetaError);
            return thetaError;
        }

        moveForward(4); // (int)1.0/(distGain)
    } while (distError > MAX_DIST_ERROR);

    return 0; // no error when we've finished
}

void Robot::turnTo(float thetaGoal, float thetaErrorLimit) {
    float theta;
    float thetaError;

    float thetaGain;

    printf("adjusting theta...\n");
    do {
		//stop();
        for (int i = 0; i < MAX_FILTER_TAPS/2; i++) {
            update();
        }
        
        theta = _pose->getTheta();
        thetaError = thetaGoal - theta;
        thetaError = Util::normalizeThetaError(thetaError);
        
		printf("theta goal: %f\n", thetaGoal);

        printf("wheel encoder x: %f\t\ty: %f\t\ttheta: %f\t\ttotal theta: %f\n", _wePose->getX(), 
                                                              _wePose->getY(), _wePose->getTheta(),
                                                              _wePose->getTotalTheta());
        printf("north star x: %f\t\ty: %f\t\ttheta: %f\t\ttotal theta: %f\n", _nsPose->getX(), 
                                                           _nsPose->getY(), _nsPose->getTheta(),
                                                           _nsPose->getTotalTheta());
        printf("kalman pose x: %f\t\ty: %f\t\ttheta: %f\t\ttotal theta: %f\n", _pose->getX(), 
                                                            _pose->getY(), _pose->getTheta(),
                                                            _pose->getTotalTheta());
        printf("theta error:\t%f\n", thetaError);

        thetaGain = _thetaPID->updatePID(thetaError);

        if (thetaError < -thetaErrorLimit) {
            printf("turning right, theta error < -limit \n");
            turnRight(5); // (int)1.0/thetaGain
        }
        else if(thetaError > thetaErrorLimit){
            printf("turning left, theta error > limit\n");
            turnLeft(5); // (int)1.0/thetaGain
        }
    } while (fabs(thetaError) > thetaErrorLimit);

    printf("theta acceptable!\n");
}

void Robot::moveForward(int speed) {
	_movingForward=true;
	printf("Moving forward\n");
	
    if (!isThereABitchInMyWay()) {
		_speed = speed;
		_robotInterface->Move(RI_MOVE_FORWARD, speed);
    }
    else {
		printf("Error, something in the way\n");
		stop();
        // TODO: remove? // turn 90 degrees
        turnTo(Util::normalizeTheta(_pose->getTheta()+DEGREE_90),
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
    
    //update the kalman constants for NS
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

    if (getStrength()>13222) { // It's OVER 9000
        //reset the theta on the we
        _wePose->setTotalTheta(_nsPose->getTotalTheta());
    }

	printf("speed: %d\n", _speed);
    if (_speed == 0) {
		printf("speed is zero\n");
        _kalmanFilter->setVelocity(0.0, 0.0, 0.0);
		printf("Kalman pose: %f, %f, %f\n",_pose->getX(), _pose->getY(), _pose->getTotalTheta());
    }
    else {
    	if(_movingForward){
    		float speedX = (_speedDistance/_forwardSpeed[_speed])/**cos(_pose->getTheta())*/;
    		float speedY = (_speedDistance/_forwardSpeed[_speed])/**sin(_pose->getTheta())*/;
			printf("Pose theta: %f\n", _pose->getTheta());
    		printf("speed: %f, %f\n", speedX, speedY);
    		_kalmanFilter->setVelocity(speedX, speedY, 0.0);
    	}
    	else {
    		printf("speed theta: %f\n", (2*PI)/_turnSpeed[_turnDirection][_speed]);
    		_kalmanFilter->setVelocity(0.0, 0.0, (2*PI)/_turnSpeed[_turnDirection][_speed]);
    	}
    }
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

    coords[1] = _getNSX();
    coords[0] = _getNSY();
    transform[0] = cos(ROOM_ROTATION[room]);
    transform[1] = -sin(ROOM_ROTATION[room]);

    if (ROOM_FLIPX[room-2]) {
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

/*
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

    coords[1] = _getNSX();
    coords[0] = _getNSY();
    transform[0] = cos(ROOM_ROTATION[room-2]);
    transform[1] = -sin(ROOM_ROTATION[room-2]);

    if (ROOM_FLIPX[room-2]) {
        transform[1] = -transform[1];
        transform[0] = -transform[0];
    }

    Util::mMult(transform, 1, 2, coords, 2, 1, &result);

    //scale
    //result /= ROOM_SCALE[0][room-2];

    //move
    float roomShiftX = COL_OFFSET[0] + ROOM_ORIGINS_FROM_COL[room-2][0];
    result += roomShiftX;

    return result;
}
*/

// Returns: transformed north star y estimate of where
// Returns: transformed north star y estimate of where
//          robot should now be in global coordinate system
// TODO?
float Robot::_getNSTransY() {
    float result;
    int room = getRoom()-2;
    float coords[2];
    float transform[2];

    coords[1] = _getNSX();
    coords[0] = _getNSY();

    transform[0] = sin(ROOM_ROTATION[room]);
    transform[1] = cos(ROOM_ROTATION[room]);

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
    if (room == ROOM_2) {
        result += .75*_getNSTransX();
    }

    return result;
}

/*
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

    coords[1] = _getNSX();
    coords[0] = _getNSY();

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
*/

// Returns: transformed north star theta estimate of where
//          robot should now be in global coordinate system
// TODO?
float Robot::_getNSTransTheta() {
    float result = _getNSTheta();
    int room = getRoom()-2;
    result -= (ROOM_ROTATION[room] * (PI/180.0));
    
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
    
    if (lastTheta > (3/2.0)*PI && newTheta < PI/2.0) {
        _wePose->modifyRotations(1);
    }
    else if (lastTheta < PI/2.0 && newTheta > (3/2.0)*PI) {
        _wePose->modifyRotations(-1);
    }

    /*
    if (_passed2PIwe && (lastTheta > PI && newTheta < PI)) {
        _wePose->modifyRotations(1);
        _passed2PIwe = false;
    }
    else if(_passed2PIwe && (lastTheta < PI && newTheta > PI)) {
        _wePose->modifyRotations(-1);
        _passed2PIwe = false;
    }
    */

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
    
    if (lastTheta > (3/2.0)*PI && newTheta < PI/2.0) {
        _nsPose->modifyRotations(1);
    }
    else if (lastTheta < PI/2.0 && newTheta > (3/2.0)*PI) {
        _nsPose->modifyRotations(-1);
    }
    
    /*
    if (_passed2PIns && (lastTheta > PI && newTheta < PI)) {
        _nsPose->modifyRotations(1);
        _passed2PIns = false;
    }
    else if (_passed2PIns && (lastTheta < PI && newTheta > PI)) {
        _nsPose->modifyRotations(-1);
        _passed2PIns = false;
    }
    */
    
    _nsPose->setX(newX);
    _nsPose->setY(newY);
    _nsPose->setTheta(newTheta);
}
