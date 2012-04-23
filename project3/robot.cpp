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
#include <unistd.h>

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

    PIDConstants movePIDConstants = {PID_MOVE_KP, PID_MOVE_KI, PID_MOVE_KD};
    PIDConstants turnPIDConstants = {PID_TURN_KP, PID_TURN_KI, PID_TURN_KD};
    PIDConstants centerTurnPIDConstants = {PID_CENTERTURN_KP, PID_CENTERTURN_KI, PID_CENTERTURN_KD};
    PIDConstants centerStrafePIDConstants = {PID_CENTERSTRAFE_KP, PID_CENTERSTRAFE_KI, PID_CENTERSTRAFE_KD};

    _movePID = new PID(&movePIDConstants, MIN_MOVE_ERROR, MAX_MOVE_ERROR);
    _turnPID = new PID(&turnPIDConstants, MIN_TURN_ERROR, MAX_TURN_ERROR);
    _centerTurnPID = new PID(&centerTurnPIDConstants, MIN_CENTERTURN_ERROR, MAX_CENTERTURN_ERROR);
    _centerStrafePID = new PID(&centerStrafePIDConstants, MIN_CENTERSTRAFE_ERROR, MAX_CENTERSTRAFE_ERROR);

    printf("pid controllers initialized\n");
    
    // Put robot head down for NorthStar use
    moveHead(RI_HEAD_DOWN);

    // fill our sensors with data
    prefillData();
    // base the wheel encoder pose off north star to start,
    // since we might start anywhere in the global system)
    _wheelEncoders->resetPose(_northStar->getPose());
    // now update our pose again so our global pose isn't
    // some funky value (since it's based off we and ns)
    updatePose(true);

    // load the map once we've found our position in the global
    // system, so we can know what cell we started at
    int startingX;
    int startingY;
    if (_pose->getX() > 150) {
        startingX = 0;
        startingY = 2;
    }
    else {
        startingX = 6;
        startingY = 2;
    }
    
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
    delete _movePID;
    delete _turnPID;
    delete _centerTurnPID;
    delete _centerStrafePID;
    delete _map;
    delete _mapStrategy;
}

/**************************************
 * Definition:	Perform the calculations for the Rovio-Man project (project 3)
 * 				get the next cell from the path created by map_strategy
 * 				and perform the 'move' function in the desired direction
 *************************************/
void Robot::eatShit() {
    Cell *nextCell = _mapStrategy->nextCell();

    while (nextCell != NULL) {
        _map->reserveCell(nextCell->x, nextCell->y);
        
        Cell *curCell = _map->getCurrentCell();

        int xDiff = nextCell->x - curCell->x;
        int yDiff = nextCell->y - curCell->y;

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

        _map->occupyCell(nextCell->x, nextCell->y);

		nextCell = _mapStrategy->nextCell();
    }
}

/**************************************
 * Definition: Moves the robot in the specified direction
 *             the specified number of cells (65x65 area)
 *
 * Parameters: ints specifying direction and number of cells
 **************************************/
void Robot::move(int direction, int numCells) {
    _heading = direction;

    int cellsTraveled = 0;
    while (cellsTraveled < numCells) {
        // make sure we're facing the right direction first
        turn(direction);
        // center ourselves between the walls
        center();
        // turn once more to fix any odd angling 
        turn(direction);
        // update our pose estimates now, ignoring
        // wheel encoders and setting them to be north star's
        updatePose(false);
        _wheelEncoders->resetPose(_northStar->getPose());
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
        // make sure we stop once we're there
		stop();
        // we made it!
        cellsTraveled++;
        LOG.write(LOG_LOW, "move", "Made it to cell %d", cellsTraveled);
    }
}

/************************************
 * Definition:	Center the robot in the North/South and East/West
 * 				directions based on the state of the cell that the
 * 				robot is in
 ***********************************/
bool Robot::sideCenter(int direction) {
    Cell *curCell = _map->getCurrentCell();
    if (curCell->getCellType() == CELL_HALL) {
        return false;
    }

    switch (direction) {
    case DIR_NORTH:
    case DIR_SOUTH:
        if (_map->canOccupy(curCell->x-1, curCell->y)) {
            turn(DIR_EAST);
			printf("Opening is: EAST\n");
        }
        else if (_map->canOccupy(curCell->x+1, curCell->y)) {
            turn(DIR_WEST);
			printf("Opening is: WEST\n");
        }
        break;
    case DIR_EAST:
    case DIR_WEST:
        if (_map->canOccupy(curCell->x, curCell->y+1)) {
            turn(DIR_NORTH);
			printf("Opening is: NORTH\n");
        }
        else if (_map->canOccupy(curCell->x, curCell->y-1)) {
            turn(DIR_SOUTH);
			printf("Opening is: SOUTH\n");
        }
        break;
    }
    center();

    return true;
}

/************************************
 * Definition:	Turns the robot in the cardinal direction
 * 				given as the parameter
 * 
 * Parameters:	A cardinal direction constant as follows
 * 				DIR_NORTH, DIR_SOUTH, DIR_EAST, DIR_WEST
 ***********************************/
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
        goalTheta = Util::normalizeTheta(_northStar->getTheta()+radians);
        turnTo(goalTheta, MAX_THETA_ERROR);
    }
    else {
        goalTheta = Util::normalizeTheta(_northStar->getTheta()-radians);
        turnTo(goalTheta, MAX_THETA_ERROR);
    }
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
		float goal = Util::normalizeTheta(_northStar->getTheta() + thetaError);
        if (thetaError != 0) {
            // if we're off in theta, turn to adjust 
            turnTo(goal, MAX_THETA_ERROR);
        }
    } while (thetaError != 0);

    _movePID->flushPID();
    _turnPID->flushPID();

    // reset wheel encoder pose to be Kalman pose since we hit our base
    _wheelEncoders->resetPose(_pose);
    _wheelEncoders->setTheta(_northStar->getTheta());
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

    float moveGain;

    printf("heading toward (%f, %f)\n", x, y);
    do {
        updatePose(true);

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
            distError = fabs(y - _pose->getY());
            break;
        case DIR_SOUTH:
            thetaDesired = DEGREE_270;
            distError = fabs(y - _pose->getY());
            break;
        case DIR_EAST:
            thetaDesired = DEGREE_0;
            distError = fabs(x - _pose->getX());
            break;
        case DIR_WEST:
            thetaDesired = DEGREE_180;
            distError = fabs(x - _pose->getX());
            break;
        }

        thetaError = thetaDesired - _northStar->getTheta();
        thetaError = Util::normalizeThetaError(thetaError);

        LOG.write(LOG_MED, "move_error_estimates",
                  "x err: %f \t y err: %f \t distance err: %f \t "
                  "theta err: %f \t theta desired: %f \t", 
                  xError,
                  yError,
                  distError,
                  thetaError,
                  thetaDesired);

        moveGain = _movePID->updatePID(distError);
        _turnPID->updatePID(thetaError);

        LOG.write(LOG_LOW, "move_gain", "move gain: %f", moveGain);

        if (fabs(thetaError) > thetaErrorLimit) {
			printf("theta error of %f too great\n", thetaError);
            return thetaError;
        }
        
        int moveSpeed = (int)(10 - 9 * moveGain);

        //int moveSpeed = (int) (10 - (9 * (fmin(1.0, (distError)/65.0))));
        moveSpeed = Util::capSpeed(moveSpeed, 10);

        LOG.write(LOG_MED, "pid_speeds", "forward speed: %d", moveSpeed);

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

    float turnGain;
 
    printf("adjusting theta\n");
    do {
	    updatePose(false);

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

        theta = _northStar->getTheta();
        thetaError = thetaGoal - theta;
        thetaError = Util::normalizeThetaError(thetaError);

        LOG.write(LOG_MED, "turn_error_estimates",
                  "theta err: %f \t theta goal: %f",
                  thetaError,
                  thetaGoal);

        turnGain = _turnPID->updatePID(thetaError);

        LOG.write(LOG_LOW, "turn_gain", "turn gain: %f", turnGain);

        if (thetaError < -thetaErrorLimit) {
            LOG.write(LOG_MED, "turn_adjust", 
                      "direction: right, since theta error < -limit");
            int turnSpeed = (int)(10 - 9 * turnGain);
            turnSpeed = Util::capSpeed(turnSpeed, 10);

            LOG.write(LOG_MED, "pid_speeds", "turn speed: %d", turnSpeed);

            turnRight(turnSpeed);
        }
        else if(thetaError > thetaErrorLimit){
            LOG.write(LOG_MED, "turn_adjust", 
                      "direction: left, since theta error > limit");
            int turnSpeed = (int)(10 - 9 * turnGain);
            turnSpeed = Util::capSpeed(turnSpeed, 10);

            LOG.write(LOG_MED, "pid_speeds", "turn speed: %d", turnSpeed);

            turnLeft(turnSpeed);
        }
    } while (fabs(thetaError) > thetaErrorLimit);


    printf("theta acceptable\n");
}

/*******************************
 * Definition: Moves the robot head (camera) to the position given as the argument
 * *****************************/
void Robot::moveHead(int position){
    _robotInterface->Move(position, 1);
    sleep(1);
    _robotInterface->Move(position, 1);
    sleep(1);
}

bool Robot::_centerTurn(float centerError) {
    bool success;

    //float centerTurnGain = _centerTurnPID->updatePID(centerError);
    //LOG.write(LOG_LOW, "centerTurn", "center error: %f", centerError);
    //LOG.write(LOG_LOW, "centerTurn", "center turn gain: %f", centerTurnGain);

    if (fabs(centerError) < MAX_TURN_CENTER_ERROR) {
        success = true;
        // we're close enough to centered, so stop adjusting
        LOG.write(LOG_LOW, "centerTurn", 
                  "Center error: |%f| < %f, stop correcting.", 
                  centerError, MAX_TURN_CENTER_ERROR);
    }
    else {
        success = false;

        int turnSpeed = (int)(10 - 5 * fabs(centerError));
        turnSpeed = Util::capSpeed(turnSpeed, 10);
        
        LOG.write(LOG_LOW, "pid_speeds", "turn speed: %d", turnSpeed);

        if (centerError < 0) {
            LOG.write(LOG_LOW, "centerTurn", "Center error: %f, move right", centerError);
            turnRight(turnSpeed);
        }
        else {
            LOG.write(LOG_LOW, "centerTurn", "Center error: %f, move left", centerError);
            turnLeft(turnSpeed);
        }

        // since the turns are generally small, we should ignore
        // wheel encoder updates for this
        _robotInterface->reset_state();
    }

    return success;
}

bool Robot::_centerStrafe(float centerError) {
    bool success;

    //float centerStrafeGain = _centerStrafePID->updatePID(centerError);
    //LOG.write(LOG_LOW, "centerStrafe", "center error: %f", centerError);
    //LOG.write(LOG_LOW, "centerStrafe", "center strafe gain: %f", centerStrafeGain);

    if (fabs(centerError) < MAX_STRAFE_CENTER_ERROR) {
        success = true;
        // we're close enough to centered, so stop adjusting
        LOG.write(LOG_LOW, "centerStrafe", 
                  "Center error: |%f| < %f, stop correcting.", 
                  centerError, MAX_STRAFE_CENTER_ERROR);
    }
    else {
        success = false;

        int strafeSpeed = (int)(10 - 5 * fabs(centerError));
        strafeSpeed = Util::capSpeed(strafeSpeed, 10);
        
        LOG.write(LOG_LOW, "pid_speeds", "strafe speed: %d", strafeSpeed);

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
    moveHead(RI_HEAD_MIDDLE);
	
	int turnAttempts = 0;

    Camera::prevTagState = -1;
    while (true) {
        bool turn = false;

        float centerError = _camera->centerError(COLOR_PINK, &turn);
        if (turn) {
            if (_centerTurn(centerError)) {
                break;
            }
            turnAttempts++;
        }
        else {
            if (_centerStrafe(centerError)) {
                break;
            }
        }
        
        if (turnAttempts > 2) {
			moveHead(RI_HEAD_DOWN);

			// make sure we are close to our desired heading, in case we didn't move correctly
			updatePose(false);

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

			turnAttempts = 0;

        	moveHead(RI_HEAD_MIDDLE);
	    }
    }

    moveHead(RI_HEAD_DOWN);

    _centerTurnPID->flushPID();
    _centerStrafePID->flushPID();
}

/**************************************
 * Definition: Updates the robot pose in terms of the global
 *             coord system with the best estimate of its position
 *             using a kalman filter
 **************************************/
void Robot::updatePose(bool useWheelEncoders) {
    // update the robot interface so wheel encoder
    // and north star have the same time-values
    _updateInterface();
    // update each pose estimate
    _northStar->updatePose();
    if (useWheelEncoders) {
    	_wheelEncoders->updatePose();
    } 
    else {
        _wheelEncoders->setTheta(_northStar->getTheta());
    }

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
                                        NS_Y_UNCERTAIN+0.05, 
                                        NS_THETA_UNCERTAIN+0.025);
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

/************************************
 * Definition:	Returns the name of the robot being used
 ***********************************/
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
        updatePose(true);
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
    int sleepLength = 300000; 
    if(speed > 6) { 
       _robotInterface->Move(RI_TURN_LEFT, 6);
       sleepLength -= 50000*(speed-6);
    } else {
       _robotInterface->Move(RI_TURN_LEFT, speed);
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
    int sleepLength = 300000; 
    if(speed > 6) { 
       _robotInterface->Move(RI_TURN_RIGHT, 6);
       sleepLength -= 50000*(speed-6);
    } else {
       _robotInterface->Move(RI_TURN_RIGHT, speed);
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
    _speed = speed;
    int sleepLength = 500000-(45000*speed);

    _robotInterface->Move(RI_MOVE_LEFT, 10);
    usleep(sleepLength);
    _robotInterface->Move(RI_STOP, 0);

    if (getName() == BENDER) {
        turnLeft(10);
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
    _speed = speed;
    int sleepLength = 500000-(45000*speed);

    _robotInterface->Move(RI_MOVE_RIGHT, 10);
    usleep(sleepLength);
    _robotInterface->Move(RI_STOP, 0);

    if (getName() == BENDER) {
        turnRight(10);
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
    sleep(1);
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
