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
    _numCellsTraveled = 0;
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

    printf("collecting initial data\n");
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

    printf("map loaded\n");
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
 * Definition:	Performs the calculations for the Rovio-Man project
 *
 * 				Continually gets a new cell to move to in the game,
 * 				and performs the move in the desired direction until
 *              the game is over.
 *************************************/
void Robot::playGame() {
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
        _numCellsTraveled++;

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
        if(_numCellsTraveled != 0) {
            // make sure we're facing the right direction first
            if (nsThetaReliable()) {
                turn(direction);
            }
            // center ourselves between the walls
            center();
            // turn once more to fix any odd angling 
            if (nsThetaReliable()) {
                turn(direction);
            }
        }

        // count how many squares we see down the hall
        moveHead(RI_HEAD_MIDDLE);
        float leftSquareCount = _camera->avgSquareCount(COLOR_PINK, IMAGE_LEFT);
        float rightSquareCount = _camera->avgSquareCount(COLOR_PINK, IMAGE_RIGHT);
        moveHead(RI_HEAD_DOWN);
        if (leftSquareCount > 3.0) {
            leftSquareCount = 3.0;
        }
        if (rightSquareCount > 3.0) {
            rightSquareCount = 3.0;
        }

        // update our pose estimates now, ignoring
        // wheel encoders and setting them to be north star's
        updatePose(false);
        _wheelEncoders->resetPose(_northStar->getPose());
        // finally, update our pose one more time so kalman isn't wonky
        updatePose(true);

        // based on the direction, move in the global coord system
        float goalX = _pose->getX();
        float goalY = _pose->getY();
        float distErrorLimit = MAX_DIST_ERROR;
        switch (direction) {
        case DIR_NORTH:
            goalY += CELL_SIZE;
            break;
        case DIR_SOUTH:
            goalY -= CELL_SIZE;
            break;
        case DIR_EAST:
            goalX += CELL_SIZE;
            if (_map->getCurrentCell()->x == 0 ||
                _map->getCurrentCell()->x == 1) {
                goalX -= 30.0;
            }
            break;
        case DIR_WEST:
            goalX -= CELL_SIZE;
            if (_map->getCurrentCell()->x == 0 ||
                _map->getCurrentCell()->x == 1) {
                goalX += 30.0;
            }
            break;
        }

        moveTo(goalX, goalY, distErrorLimit);
        // make sure we stop once we're there
        stop();

        moveHead(RI_HEAD_MIDDLE);
        float newLeftSquareCount = _camera->avgSquareCount(COLOR_PINK, IMAGE_LEFT);
        float newRightSquareCount = _camera->avgSquareCount(COLOR_PINK, IMAGE_RIGHT);
        if (newLeftSquareCount > 3.0) {
            newLeftSquareCount = 3.0;
        }
        if (newRightSquareCount > 3.0) {
            newRightSquareCount = 3.0;
        }
        int i = 0;
        while ((newLeftSquareCount + 1.0 < leftSquareCount ||
                newRightSquareCount + 1.0 < rightSquareCount) &&
                i < 5) {
            moveBackward(10);
            _robotInterface->reset_state();
            newLeftSquareCount = _camera->avgSquareCount(COLOR_PINK, IMAGE_LEFT);
            newRightSquareCount = _camera->avgSquareCount(COLOR_PINK, IMAGE_RIGHT);
            if (newLeftSquareCount + 1.0 < leftSquareCount &&
                newRightSquareCount + 1.0 < rightSquareCount) {
                i++;
            }     
            i++;
        }
        moveHead(RI_HEAD_DOWN);

        // we made it!
        cellsTraveled++;
        printf("Made it to cell %d\n", cellsTraveled);
    }
}

/************************************
 * Definition:	Center the robot in the North/South and East/West
 * 				directions based on the state of the cell that the
 * 				robot is in
 ***********************************/
bool Robot::sideCenter(int direction) {
    Cell *curCell = _map->getCurrentCell();

    switch (direction) {
    case DIR_NORTH:
    case DIR_SOUTH:
        if (_map->canOccupy(curCell->x-1, curCell->y)) {
            turn(DIR_EAST);
        }
        else if (_map->canOccupy(curCell->x+1, curCell->y)) {
            turn(DIR_WEST);
        }
        break;
    case DIR_EAST:
    case DIR_WEST:
        if (_map->canOccupy(curCell->x, curCell->y+1)) {
            turn(DIR_NORTH);
        }
        else if (_map->canOccupy(curCell->x, curCell->y-1)) {
            turn(DIR_SOUTH);
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
void Robot::moveTo(float x, float y, float distErrorLimit) {
    printf("beginning move\n");

    float thetaError;
    do {
        // move to the location until theta is off by too much
        thetaError = moveToUntil(x, y, distErrorLimit, MAX_THETA_ERROR);
		float goal = Util::normalizeTheta(_northStar->getTheta() + thetaError);
        if (thetaError != 0) {
            // if we're off in theta, turn to adjust 
            turnTo(goal, MAX_THETA_ERROR);
        }
    } 
    while (thetaError != 0);

    _movePID->flushPID();
    _turnPID->flushPID();

    // reset wheel encoder pose to be Kalman pose since we hit our base
    _wheelEncoders->resetPose(_pose);
    _wheelEncoders->setTheta(_northStar->getTheta());

    printf("done moving\n");
}

/**************************************
 * Definition: Attempts to move the robot to the specified location 
 *             in the global coord system, disregarding cells. If
 *             theta error is exceeded, the method returns theta error.
 *
 * Parameters: floats specifying x and y in global system, and 
 *             a float specifying the theta error limit
 **************************************/
float Robot::moveToUntil(float x, float y, float distErrorLimit, float thetaErrorLimit) {
    float yError;
    float xError;
    float thetaDesired;
    float thetaError;
    float distError;

    float moveGain;

    printf("heading toward (%f, %f)\n", x, y);
    do {
        updatePose(true);

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

        moveGain = _movePID->updatePID(distError);
        _turnPID->updatePID(thetaError);

        if (fabs(thetaError) > thetaErrorLimit) {
			printf("theta error of %f too great\n", thetaError);
            return thetaError;
        }
        
        int moveSpeed = (int)(10 - 9 * moveGain);
        moveSpeed = Util::capSpeed(moveSpeed, 10);

        moveForward(moveSpeed);
    } 
    while (distError > distErrorLimit);

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

        theta = _northStar->getTheta();
        thetaError = thetaGoal - theta;
        thetaError = Util::normalizeThetaError(thetaError);

        turnGain = _turnPID->updatePID(thetaError);

        if (thetaError < -thetaErrorLimit) {
            int turnSpeed = (int)(10 - 9 * turnGain);
            turnSpeed = Util::capSpeed(turnSpeed, 10);

            turnRight(turnSpeed);
        }
        else if(thetaError > thetaErrorLimit){
            int turnSpeed = (int)(10 - 9 * turnGain);
            turnSpeed = Util::capSpeed(turnSpeed, 10);

            turnLeft(turnSpeed);
        }
    } 
    while (fabs(thetaError) > thetaErrorLimit);

    printf("theta acceptable\n");
}

/*************************************
 * Definition: Determines what speed of turn to make based on a measure of center error
 *
 * Parameters: float value of centerError
 * 
 * Returns: boolean value false if a move was made, true if corrections not required
 ************************************/
bool Robot::_centerTurn(float centerError) {
    bool success;

    if (fabs(centerError) < MAX_TURN_CENTER_ERROR) {
        success = true;
        // we're close enough to centered, so stop adjusting
        printf("Center error: |%f| < %f, stop correcting.\n", 
               centerError, 
               MAX_TURN_CENTER_ERROR);
    }
    else {
        //We needed to make a move, so centering not YET successful
        success = false;
        
        // Not using a PID
        // Corrections are not inherently linked in time, so use of a PID isn't exactly proper
        int turnSpeed = (int)(10 - 5 * fabs(centerError));
        turnSpeed = Util::capSpeed(turnSpeed, 10);

        if (centerError < 0) {
            printf("Center error: %f, move right\n", centerError);
            turnRight(turnSpeed);
        }
        else {
            printf("Center error: %f, move left\n", centerError);
            turnLeft(turnSpeed);
        }

        // since the turns are generally small, we should ignore
        // wheel encoder updates for this
        _robotInterface->reset_state();
    }

    return success;
}

/*************************************
 * Definition: Determines what speed of strafe to make based on a measure of center error
 *
 * Parameters: float value of centerError
 * 
 * Returns: boolean value false if a move was made, true if corrections not required
 ************************************/
bool Robot::_centerStrafe(float centerError) {
    bool success;

    if (fabs(centerError) < MAX_STRAFE_CENTER_ERROR) {
        success = true;
        // we're close enough to centered, so stop adjusting
        printf("Center error: |%f| < %f, stop correcting.\n", 
               centerError, 
               MAX_STRAFE_CENTER_ERROR);
    }
    else {
        //We needed to make a move, so centering not YET successful
        success = false;

        // Not using a PID
        // Corrections are not inherently linked in time, so use of a PID isn't exactly proper
        int strafeSpeed = (int)(10 - 5 * fabs(centerError));
        strafeSpeed = Util::capSpeed(strafeSpeed, 10);
        
        if (centerError < 0) {
            printf("Center error: %f, move right\n", centerError);
            strafeRight(strafeSpeed);
        }
        else {
            printf("Center error: %f, move left\n", centerError);
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
            float speedX = SPEED_FORWARD[_speed];
            float speedY = SPEED_FORWARD[_speed];

            _kalmanFilter->setVelocity(speedX, speedY, 0.0);
        }
        else {
            float speedTheta = SPEED_TURN[_turnDirection][_speed]; //Fetch turning speed in radians per second

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
}

/************************************
 * Definition: Returns the robot interface
 ***********************************/
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
    printf("prefilling data\n");
    for (int i = 0; i < MAX_FILTER_TAPS; i++){
        updatePose(true);
    }
    printf("sufficient data collected\n");
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
 * Definition: Moves the robot backward, keeping track of movement.
 *             (Wrapper around robot interface)
 *
 * Parameters: int specifying speed to move at
 **************************************/
void Robot::moveBackward(int speed) {
    _movingForward = false;
    _speed = speed;
    _robotInterface->Move(RI_MOVE_BACKWARD, speed);
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

    // no robot strafes nicely, so turn a bit to fix it
    turnLeft(10);
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

    // no robot strafes nicely, so turn a bit to fix it
    turnRight(10);
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
 * Definition: Specifies whether the North Star theta value can be trusted based on call location
 *
 * Returns:    bool value specifying validity of NS theta data
 *************************************/
bool Robot::nsThetaReliable() {
    int x = _map->getCurrentCell()->x;
    int y = _map->getCurrentCell()->y;

    if (y == 0 && x < 5) {
        return false;
    }
    else {
        return true;
    }
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
