/**
 * robot.cpp
 * 
 * @brief 
 * 		This class defines the rovio robot, and performs operations that the 
 *      system can do, such as movement and requesting sensor updates.  
 *      It also stores the sensor classes, vehicle position, and camera.
 * 
 * @author
 *      Shawn Hanna
 *      Tom Nason
 *      Joel Griffith
 * 
 **/

#include "robot.h"
#include "phrases.h"
#include "logger.h"
#include <math.h>

Robot::Robot(std::string address, int id) {
    // store the robot's name as an int to be used later
    _name = Util::nameFrom(address);
    
    // initialize movement variables
    _turnDirection = 0;
    _movingForward = true;
    _speed = 0;
    _heading = DIR_NORTH;

    setFailLimit(MAX_UPDATE_FAILS);

    _robotInterface = new RobotInterface(address, id);

    printf("robot interface loaded\n");

    // initialize camera
    _camera = new Camera(_robotInterface);

    // initialize position sensors
    _wheelEncoders = new WheelEncoders(this);
    _northStar = new NorthStar(this);
    
    // initialize global pose
    _pose = new Pose(0.0, 0.0, 0.0);
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

    _distancePID = new PID(&distancePIDConstants, MIN_DIST_GAIN, MAX_DIST_GAIN);
    _thetaPID = new PID(&thetaPIDConstants, MIN_THETA_GAIN, MAX_THETA_GAIN);
    _centerPID = new PID(&centerPIDConstants, MIN_CENTER_GAIN, MAX_CENTER_GAIN);
    _turnCenterPID = new PID(&turnCenterPIDConstants, MIN_TURN_CENTER_GAIN, MAX_TURN_CENTER_GAIN);

    printf("pid controllers initialized\n");
    
    // fill our sensors with data
    prefillData();
    // base the wheel encoder pose off north star to start,
    // since we might start anywhere in the global system)
    _wheelEncoders->resetPose(_northStar->getPose());
    // now update our pose again so our global pose isn't
    // some funky value (since it's based off we and ns)
    updatePose();

    // load the map once we've found our position in the global
    // system, so we can know what cell we started at
    // FIXME
    int startingX = ((int)_pose->getX()) / CELL_SIZE;
    int startingY = ((int)_pose->getY()) / CELL_SIZE;

    _map = new Map(_robotInterface, startingX, startingY);
    _mapStrategy = new MapStrategy(_map);
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
    delete _turnCenterPID;
    delete _map;
    delete _mapStrategy;
}

void Robot::playGame() {
    Cell *nextCell = _mapStrategy->nextCell();

    while (nextCell != NULL) {
        Cell *curCell = _map->getCurrentCell();
        
        int xDiff = nextCell->x - curCell->x;
        int yDiff = nextCell->y - curCell->y;

        // TODO: make sure these match up with proper cardinal
        // directions
        if (xDiff > 0) {
            move(DIR_WEST, 1);
        }
        else if (xDiff < 0) {
            move(DIR_EAST, 1);
        }
        else if (yDiff > 0) {
            move(DIR_NORTH, 1);
        }
        else if (yDiff < 0) {
            move(DIR_SOUTH, 1);
        }
    }
}

/**************************************
 * Definition: Moves the robot in the specified direction
 *             the specified number of cells (65x65 area)
 *
 * Parameters: ints specifying direction and number of cells
 **************************************/
void Robot::move(int direction, int numCells) {
    // make sure we're facing the right direction first
    turn(direction);

    _heading = direction;

    int cellsTraveled = 0;
    while (cellsTraveled < numCells) {
        // first attempt to center ourselves before moving (except not first)
        center();
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
        moveTo(goalX, goalY); // was moveToCell
        cellsTraveled++;
        LOG.write(LOG_LOW, "move", "Made it to cell %d", cellsTraveled);
    }
}

void Robot::turn(int direction) {
    switch (direction) {
    case DIR_NORTH:
        turnTo(DEGREE_90, MAX_THETA_ERROR);
        break;
    case DIR_SOUTH:
        turnTo(DEGREE_270, MAX_THETA_ERROR);
        break;
    case DIR_EAST:
        turnTo(DEGREE_0, MAX_THETA_ERROR);
        break;
    case DIR_WEST:
        turnTo(DEGREE_180, MAX_THETA_ERROR);
        break;
    }
}

/**************************************
 * Definition: Turns the robot in a relative direction
 *             the specified number of radians
 *
 * Parameters: int specifying direction and a float
 *             specifying radians (0..2PI)
 **************************************/
void Robot::turn(int relDirection, float radians) {
    float goalTheta;

    if (relDirection == DIR_LEFT) {
        goalTheta = Util::normalizeTheta(_pose->getTheta()+radians);
        turnTo(goalTheta, MAX_THETA_ERROR);
    }
    else {
        goalTheta = Util::normalizeTheta(_pose->getTheta()-radians);
        turnTo(goalTheta, MAX_THETA_ERROR);
    }
}

/**************************************
 * Definition: Moves the robot to a cell in the global coord system located at
 *             the specified x and y, centering as needed
 *
 * Parameters: floats specifying x and y in global system
 **************************************/
void Robot::moveToCell(float x, float y) {
    LOG.write(LOG_LOW, "moveToCell", 
              "moveToCell cur. location: %f, %f, %f", 
              _pose->getX(), _pose->getY(), _pose->getTheta());

    printf("beginning move to cell at (%f, %f)\n", x, y);

    float thetaError;
    do {
        // move to the location until theta is off by too much
        thetaError = moveToUntil(x, y, MAX_THETA_ERROR);
        float goal = Util::normalizeTheta(_pose->getTheta() + thetaError);
        if (thetaError != 0) {
            // if we're off in theta, turn to adjust 
            turnTo(goal, MAX_THETA_ERROR);
            // finally, center between squares as a sanity check
            // turnCenter();
        }
    } while (thetaError != 0);

    _distancePID->flushPID();
    _thetaPID->flushPID();

    // reset wheel encoder pose to be Kalman pose since we hit our base
    _wheelEncoders->resetPose(_pose);
}

/**************************************
 * Definition: Moves the robot to the specified location in the global
 *             coord system, disregarding cells
 *
 * Parameters: floats specifying x and y in global system
 **************************************/
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

/**************************************
 * Definition: Attempts to move the robot to the specified location 
 *             in the global coord system, disregarding cells. If
 *             theta error is exceeded, the method returns theta error.
 *
 * Parameters: floats specifying x and y in global system, and 
 *             a float specifying the theta error limit
 **************************************/
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

        LOG.write(LOG_HIGH, "move_we_pose",
                  "x: %f \t y: %f \t theta: %f", 
                  _wheelEncoders->getPose()->getX(),
                  _wheelEncoders->getPose()->getY(),
                  _wheelEncoders->getPose()->getTheta());
        LOG.write(LOG_HIGH, "move_ns_pose",
                  "x: %f \t y: %f \t theta: %f", 
                  _northStar->getPose()->getX(),
                  _northStar->getPose()->getY(),
                  _northStar->getPose()->getTheta()); 
        LOG.write(LOG_HIGH, "move_kalman_pose",
                  "x: %f \t y: %f \t theta: %f", 
                  _pose->getX(),
                  _pose->getY(),
                  _pose->getTheta()); 

        yError = y - _pose->getY();
        xError = x - _pose->getX();

        switch (_heading) {
        case DIR_NORTH:
            thetaDesired = DEGREE_90;
        case DIR_SOUTH:
            thetaDesired = DEGREE_270;
            distError = fabs(y - _pose->getY());
            break;
        case DIR_EAST:
            thetaDesired = DEGREE_0;
        case DIR_WEST:
            thetaDesired = DEGREE_180;
            distError = fabs(x - _pose->getX());
            break;
        }

        thetaError = thetaDesired - _pose->getTheta();
        thetaError = Util::normalizeThetaError(thetaError);

/*      Old way

        thetaDesired = atan2(yError, xError);
        thetaDesired = Util::normalizeTheta(thetaDesired);

        distError = sqrt(yError*yError + xError*xError);
*/

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

/**************************************
 * Definition: Turns the robot as close to the specified theta
 *             as possible. When theta is within the specified
 *             theta error threshold, it returns.
 *
 * Parameters: floats specifying theta goal and theta error limit
 **************************************/
void Robot::turnTo(float thetaGoal, float thetaErrorLimit) {
    float theta;
    float thetaError;

    float thetaGain;
 
    printf("adjusting theta\n");
    do {
	    updatePose();

        LOG.write(LOG_LOW, "turn_we_pose",
                  "x: %f \t y: %f \t theta: %f", 
                  _wheelEncoders->getPose()->getX(),
                  _wheelEncoders->getPose()->getY(),
                  _wheelEncoders->getPose()->getTheta()); 
        LOG.write(LOG_LOW, "turn_ns_pose",
                  "x: %f \t y: %f \t theta: %f", 
                  _northStar->getPose()->getX(),
                  _northStar->getPose()->getY(),
                  _northStar->getPose()->getTheta()); 
        LOG.write(LOG_LOW, "turn_kalman_pose",
                  "x: %f \t y: %f \t theta: %f", 
                  _pose->getX(),
                  _pose->getY(),
                  _pose->getTheta()); 

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

// Deprecated
/**************************************
 * Definition: Turns the robot until it is considered centered
 *             between two squares in a corridor
 **************************************/
 /*
void Robot::turnCenter() {
    while (true) {
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
*/

bool Robot::_centerTurn(float centerError) {
    bool success;

    float centerGain = _turnCenterPID->updatePID(centerError);
    LOG.write(LOG_LOW, "centerTurn", "center error: %f", centerError);
    LOG.write(LOG_LOW, "centerTurn", "center gain: %f", centerGain);

    if (fabs(centerError) < MAX_TURN_CENTER_ERROR) {
        success = true;
        // we're close enough to centered, so stop adjusting
        LOG.write(LOG_LOW, "centerTurn", 
                  "Center error: |%f| < %f, stop correcting.", 
                  centerError, MAX_TURN_CENTER_ERROR);
    }
    else {
        success = false;

        int turnSpeed = (int)fabs((centerGain));
        turnSpeed = Util::capSpeed(turnSpeed, 8);
        
        LOG.write(LOG_LOW, "pid_speeds", "turn: %d", turnSpeed);

        if (centerError < 0) {
            LOG.write(LOG_LOW, "centerTurn", "Center error: %f, move right", centerError);
            turnRight(turnSpeed);
        }
        else {
            LOG.write(LOG_LOW, "centerTurn", "Center error: %f, move left", centerError);
            turnLeft(turnSpeed);
        }

        // make sure we haven't turned beyond 45 degrees (our max adjustment)
        updatePose();

        float thetaHeading;
        switch (_heading) {
        case DIR_NORTH:
            thetaHeading = DEGREE_90;
            break;
        case DIR_SOUTH:
            thetaHeading = DEGREE_270;
            break;
        case DIR_EAST:
            thetaHeading = DEGREE_0;
            break;
        case DIR_WEST:
            thetaHeading = DEGREE_180;
            break;
        }

        float theta = _pose->getTheta();
        float thetaError = thetaHeading - theta;
        thetaError = Util::normalizeThetaError(thetaError);
        if (fabs(thetaError) > DEGREE_45) {
            // if we turned beyond 45 degrees from our
            // heading, this was a mistake. let's turn
            // back just enough so we're 45 degrees from
            // our heading.
            float thetaGoal = Util::normalizeTheta(theta + (DEGREE_45 + thetaError));
            turnTo(thetaGoal, MAX_THETA_ERROR);
        }
    }

    return success;
}

bool Robot::_centerStrafe(float centerError) {
    bool success;

    float centerGain = _centerPID->updatePID(centerError);
    LOG.write(LOG_LOW, "centerStrafe", "center error: %f", centerError);
    LOG.write(LOG_LOW, "centerStrafe", "center gain: %f", centerGain);

    if (fabs(centerError) < MAX_CENTER_ERROR) {
        success = true;
        // we're close enough to centered, so stop adjusting
        LOG.write(LOG_LOW, "centerStrafe", 
                  "Center error: |%f| < %f, stop correcting.", 
                  centerError, MAX_CENTER_ERROR);
    }
    else {
        success = false;

        int strafeSpeed = (int)fabs((centerGain));
        strafeSpeed = Util::capSpeed(strafeSpeed, 8);
        
        LOG.write(LOG_LOW, "pid_speeds", "strafe: %d", strafeSpeed);

        if (centerError < 0) {
            LOG.write(LOG_LOW, "centerStrafe", "Center error: %f, move right", centerError);
            strafeRight(strafeSpeed);
        }
        else {
            LOG.write(LOG_LOW, "centerStrafe", "Center error: %f, move left", centerError);
            strafeLeft(strafeSpeed);
        }

        // we moved, so reset the wheel encoders to ignore
        // this movement
        _robotInterface->reset_state();
    }

    return success;
}

/**************************************
 * Definition: Strafes the robot until it is considered centered
 *             between two squares in a corridor
 **************************************/
void Robot::center() {
    while (true) {
        updateCamera();

        bool turn = false;
        float centerError = _camera->centerError(COLOR_PINK, &turn);
        if (turn) {
            if (_centerTurn(centerError)) {
                break;
            }
        }
        else {
            if (_centerStrafe(centerError)) {
                break;
            }
        }
    }

    _centerPID->flushPID();
    _turnCenterPID->flushPID();
}

/**************************************
 * Definition: Updates the robot's camera, reading in a new image.
 **************************************/
void Robot::updateCamera() {
    //Put robot head up for camera use
    _robotInterface->Move(RI_HEAD_MIDDLE, 1);
    _camera->update();
    //Put robot head down for NorthStar use
    _robotInterface->Move(RI_HEAD_DOWN, 1);
}

/**************************************
 * Definition: Updates the robot pose in terms of the global
 *             coord system with the best estimate of its position
 *             using a kalman filter
 **************************************/
void Robot::updatePose() {
    // update the robot interface so wheel encoder
    // and north star have the same time-values
    _updateInterface();
    // update each pose estimate
    _northStar->updatePose();
    _wheelEncoders->updatePose();

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

    LOG.write(LOG_LOW, "position_data", "Room:\t%d\tNS:\t%f\t%f\t%f\tWE:\t%f\t%f\t%f\tKalman:\t%f\t%f\t%f\t", getRoom(), _northStar->getX(), _northStar->getY(), _northStar->getTheta(), _wheelEncoders->getX(), _wheelEncoders->getY(), _wheelEncoders->getTheta(), _pose->getX(), _pose->getY(), _pose->getTheta());
}

RobotInterface* Robot::getInterface() {
    return _robotInterface;
}

int Robot::getName() {
    return _name;
}

/**************************************
 * Definition: Returns a reference to the Kalman pose
 *
 * Returns:    a pointer to a pose
 **************************************/
Pose* Robot::getPose() {
    return _pose;
}

/**************************************
 * Definition: Fills sensor filters entirely
 **************************************/
void Robot::prefillData() {
    printf("prefilling data...\n");
    for (int i = 0; i < MAX_FILTER_TAPS; i++){
        updatePose();
    }
    printf("sufficient data collected\n");
}

/**************************************
 * Definition: Moves the robot forward, keeping track of movement.
 *             (Wrapper around robot interface)
 *
 * Parameters: int specifying speed to move at
 **************************************/
void Robot::moveForward(int speed) {
	_movingForward = true;
    _speed = speed;
    _robotInterface->Move(RI_MOVE_FORWARD, speed);
}

/**************************************
 * Definition: Turns the robot left at the given speed.
 *             (Wrapper around robot interface)
 *
 * Parameters: int specifying speed to turn at
 **************************************/
void Robot::turnLeft(int speed) {
	_turnDirection = DIR_LEFT;
	_movingForward = false;
	_speed = speed;
    _robotInterface->Move(RI_TURN_LEFT, speed);
    int sleepLength = 0; 
    for (int i = speed; i > 0; i--) {
        sleepLength += 1000*100; // 1/10th of a second
    }
    usleep(sleepLength);
    _robotInterface->Move(RI_STOP, 0);
}

/**************************************
 * Definition: Turns the robot right at the given speed.
 *             (Wrapper around robot interface)
 *
 * Parameters: int specifying speed to turn at
 **************************************/
void Robot::turnRight(int speed) {
	_turnDirection = DIR_RIGHT;
	_movingForward = false;
	_speed = speed;
    _robotInterface->Move(RI_TURN_RIGHT, speed);
    int sleepLength = 0; 
    for (int i = speed; i > 0; i--) {
        sleepLength += 1000*100; // 1/10th of a second
    }
    usleep(sleepLength);
    _robotInterface->Move(RI_STOP, 0);
}

/**************************************
 * Definition: Strafes the robot left at the given speed.
 *             (Wrapper around robot interface)
 *
 * Note:       Since strafing sideways is difficult, the
 *             command is sent multiple times to scale with
 *             the speed in order to avoid turning the robot.
 *
 * Parameters: int specifying speed to strafe at
 **************************************/
void Robot::strafeLeft(int speed) {
	_speed = 10;
    // we need about 5 commands to actually strafe sideways
    for (int i = 0; i < 5; i++) {
        _robotInterface->Move(RI_MOVE_LEFT, 10);
    }
}

/**************************************
 * Definition: Strafes the robot right at the given speed.
 *             (Wrapper around robot interface)
 *
 * Note:       Since strafing sideways is difficult, the
 *             command is sent multiple times to scale with
 *             the speed in order to avoid turning the robot.
 *
 * Parameters: int specifying speed to strafe at
 **************************************/
void Robot::strafeRight(int speed) {
	_speed = 10;
    for (int i = 0; i < 5; i++) {
        _robotInterface->Move(RI_MOVE_RIGHT, 10);
    }
}

/**************************************
 * Definition: Stops the robot from moving, updating movement variables.
 *             (Wrapper around robot interface)
 **************************************/
void Robot::stop() {
	_movingForward = true;
	_speed = 0;
    _robotInterface->Move(RI_STOP, 0);
}

/**************************************
 * Definition: Returns the North Star room the robot is in
 *
 * Returns:    int specifying the room (starting at 0)
 **************************************/
int Robot::getRoom() {
    return _robotInterface->RoomID() - 2;
}

/**************************************
 * Definition: Returns the robot's battery level.
 *             (Wrapper around robot interface)
 *
 * Returns:    int specifying battery level
 **************************************/
int Robot::getBattery() {
    return _robotInterface->Battery();
}

/**************************************
 * Definition: Returns the robot's battery level.
 *             (Wrapper around robot interface)
 *
 * Returns:    int specifying battery level
 **************************************/
int Robot::getStrength(){
    return _robotInterface->NavStrengthRaw();
}

/**************************************
 * Definition: Returns the status of obstruction for the robot.
 *             (Wrapper around robot interface)
 *
 * Returns:    bool specifying if the robot is blocked or not
 **************************************/
bool Robot::isThereABitchInMyWay() {
    return _robotInterface->IR_Detected();
}

/**************************************
 * Definition: Attempts to update the robot interface a certain
 *             amount of times. Returns true on success.
 *
 * Returns:    bool specifying if we succeeded
 **************************************/
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

/**************************************
 * Definition: Sets the amount of times we can fail at
 *             updating the robot interface before stopping.
 *
 * Parameters: int specifying the limit
 **************************************/
void Robot::setFailLimit(int limit) {
    _failLimit = limit;
}

/**************************************
 * Definition: Returns the fail limit
 *
 * Returns:    int specifying the limit
 **************************************/
int Robot::getFailLimit() {
    return _failLimit;
}

/**************************************
 * Definition: Sets the camera resolution and quality
 *             (Wrapper around camera)
 *
 * Parameters: ints specifying resolution and quality
 **************************************/
void Robot::setCameraResolution(int resolution, int quality) {
    _camera->setResolution(resolution);
    _camera->setQuality(quality);
}

/**************************************
 * Definition: Prints out the robot's beginning phrase
 **************************************/
void Robot::printBeginPhrase() {
    printf(BEGIN_PHRASES[_name].c_str());
    printf("\n");
}

/**************************************
 * Definition: Prints out the robot's success phrase
 **************************************/
void Robot::printSuccessPhrase() {
    printf(SUCCESS_PHRASES[_name].c_str());
    printf("\n");
}

/**************************************
 * Definition: Prints out the robot's failure phrase
 **************************************/
void Robot::printFailPhrase() {
    printf(FAIL_PHRASES[_name].c_str());
    printf("\n");
}

/**************************************
 * Definition: Celebrates by moving the robot's head up and down
 **************************************/
void Robot::rockOut() {
    for (int i = 0; i < 1; i++) {
        _robotInterface->Move(RI_HEAD_UP, 1);
        sleep(1);
        _robotInterface->Move(RI_HEAD_DOWN, 1);
        sleep(1);
    }
}
