#include "robot.h"
#include <math.h>

#define MAX_NUM_FAILS 5

#define MIN_DIST_GAIN -0.1
#define MAX_DIST_GAIN 0.1
#define MIN_THETA_GAIN -0.05
#define MAX_THETA_GAIN 0.05

#define PID_DIST_KP 0.8
#define PID_DIST_KI 0.05
#define PID_DIST_KD 0.05

#define PID_THETA_KP 0.65
#define PID_THETA_KI 0.001
#define PID_THETA_KD 0.001

#define MAX_FILTER_TAPS 7

#define MAX_THETA_ERROR 0.5236 // 30 degrees
// .26 // 15 degrees
#define MAX_DIST_ERROR 30.0 // in cm

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

    prefillData();

    printf("pid controllers initialized\n");
}

int Robot::nameToInt(){
    if(name=="Rosie" || name=="rosie")
        return 1;
    if(name=="Bender" || name=="bender")
        return 2;
    if(name=="Johnny5" || name=="johnny5")
        return 3;
    if(name=="Optimus" || name=="optimus")
        return 4;
    if(name=="Walle" || name=="walle" || name=="wallE" || name=="WallE")
        return 5;
    if(name=="Gort" || name=="gort")
        return 5;
    return -1;
}

void Robot::printOpeningDialog(){
    printf("\n\n\n");
    switch (nameToInt()){
        case 1:
            printf("I swear on my mother's rechargable batteries\n");
            break;
        case 2:
            printf("Bender here\n");
            break;
        case 3:
            printf("I'm Johnny5\n");
            break;
        case 4:
            printf("Transformers, robots in disguise\n");
            break;
        case 5:
            printf("WallEEEEEEEEeeeeeeee\n");
            break;
        default:
            printf("I don't know who I am\n");
            break;
    }
    printf("\n\n\n");
}
void Robot::printFailureDialog(){
    printf("\n\n\n");
    switch (nameToInt()){
        case 1:
            printf("Again?\n");
            break;
        case 2:
            printf("I just messed up\n");
            break;
        case 3:
            printf("I'm Johnny5, and I crashed\n");
            break;
        case 4:
            printf("In war, there are calms between storms.\nThere will be days when we lose faith, days when our allies turn against us.");
            break;
        case 5:
            printf("WallE...\n");
            break;
        default:
            printf("I don't do anything right... :(");
            break;
    }
    printf("\n\n\n");
}
void Robot::printSuccessDialog(){
    printf("\n\n\n");
    switch (nameToInt()){
        case 1:
            printf("Again?\n");
            break;
        case 2:
            printf("I'm awesome\n");
            break;
        case 3:
            printf("I'm Johnny5, and I did not go insane\n");
            break;
        case 4:
            printf("There's more than meets the eye");
            break;
        case 5:
            printf("WallE...\n");
            break;
        default:
            printf("Good JOB!!!!!!!!!!!!!!!");
            break;
    }
    printf("\n\n\n");
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

    do {
        thetaError = moveToUntil(x, y, MAX_THETA_ERROR);
        if (thetaError != 0) {
            turnTo(_pose->getTheta()+thetaError, MAX_THETA_ERROR); // FIXME: should be -?
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

    printf("heading toward %f, %f\n", x, y);
    do {
        update();

        printf("wheel encoder x: %f\t\ty: %f\t\ttheta: %f\n", _wePose->getX(), 
                                                              _wePose->getY(), 
                                                              _wePose->getTheta());
        printf("north star x: %f\t\ty: %f\t\ttheta: %f\n", _nsPose->getX(), 
                                                           _nsPose->getY(), 
                                                           _nsPose->getTheta());
        printf("kalman pose x: %f\t\ty: %f\t\ttheta: %f\n", _pose->getX(), 
                                                            _pose->getY(), 
                                                            _pose->getTheta());

        yError = y - _pose->getY();
        xError = x - _pose->getX();
        /*
        // previous way of calculating desired
        thetaDesired = atan2(yError, xError);
        thetaDesired = (float)(fmod(2*PI+thetaDesired, 2*PI));
        */
        thetaDesired = acos(xError / (sqrt(yError*yError + xError*xError)));
        if (yError < 0) {
            thetaDesired += PI;
        }
        printf("desired theta: %f\n", thetaDesired);

        thetaError = thetaDesired - _pose->getTheta();
        if (thetaError > PI) {
            thetaError = -(2*PI-thetaError);
        }
        
        distError = sqrt(yError*yError + xError*xError);

        // TODO: remove
        printf("x error:\t%f\n", xError);
        printf("y error:\t%f\n", yError);
        printf("theta error:\t%f\n", thetaError);
        printf("distance error:\t%f\n", distError);

        distGain = _distancePID->updatePID(distError);
        _thetaPID->updatePID(thetaError);

        if ((fabs(thetaError) > thetaErrorLimit) && xError < MAX_DIST_ERROR+20) {
            return thetaError;
        }

        moveForward(5); // (int)1.0/(distGain)
    } while (distError > MAX_DIST_ERROR);

    return 0; // no error when we've finished
}

void Robot::turnTo(float thetaGoal, float thetaErrorLimit) {
    float theta;
    float thetaError;

    float thetaGain;

    printf("adjusting theta...\n");
    do {
        for (int i = 0; i < MAX_FILTER_TAPS/2; i++) {
            update();
        }

        theta = _pose->getTheta();
        thetaError = theta - thetaGoal;

        printf("wheel encoder x: %f\t\ty: %f\t\ttheta: %f\n", _wePose->getX(), 
                                                              _wePose->getY(), 
                                                              _wePose->getTheta());
        printf("north star x: %f\t\ty: %f\t\ttheta: %f\n", _nsPose->getX(), 
                                                           _nsPose->getY(), 
                                                           _nsPose->getTheta());
        printf("kalman pose x: %f\t\ty: %f\t\ttheta: %f\n", _pose->getX(), 
                                                            _pose->getY(), 
                                                            _pose->getTheta());
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
    if (!isThereABitchInMyWay()) {
        _robotInterface->Move(RI_MOVE_FORWARD, speed);
    }
    else {
        turnRight(5);  
        if (name == "Optimus") {
            printf("No No No No No No No!!!\t\tDETECTION!");
        }
    }
}

void Robot::turnLeft(int speed) {
    _robotInterface->Move(RI_TURN_LEFT, speed);
}

void Robot::turnRight(int speed) {
    _robotInterface->Move(RI_TURN_RIGHT, speed);
}

void Robot::stop() {
    _robotInterface->Move(RI_STOP, 0);
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
    if (newY > 0.3)
        newY = .3;
    }
    if (newTheta > 0.3) {
        newTheta = .3;
    }

    //_kalmanFilter->setNSUncertainty(.15, .15, .08);
    //printf("certainties: %f %f %f\n", newX, newY, newTheta);
    //update the kalman constants for WE

    if (getStrength()>13222) { // It's OVER 9000
        //reset the theta on the we
        _wePose->setTheta(_nsPose->getTheta());
        _wePose->setNumRotations(_nsPose->getNumRotations());
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
    int room = _robotInterface->RoomID();
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
    result /= ROOM_SCALE[0][room-2];

    //move
    result += ROOM_X_SHIFT[room-2];

    return result;
}

// TEMP FUNCTION!
//
// Returns: transformed north star x estimate of where
//          robot should now be in global coordinate system
// TODO?
float Robot::_getNSHalfTransX() {
    float result;
    int room = _robotInterface->RoomID();
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
    //result += ROOM_X_SHIFT[room-2];

    return result;
}

// Returns: transformed north star y estimate of where
// Returns: transformed north star y estimate of where
//          robot should now be in global coordinate system
// TODO?
float Robot::_getNSTransY() {
    float result;
    int room = _robotInterface->RoomID();
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
    result /= ROOM_SCALE[1][room-2];

    //move
    result += ROOM_Y_SHIFT[room-2];

    //Correction for skew in room 2
    if (room == 2) {
        result += .75*_getNSTransX();
    }

    return result;
}

// TEMP FUNCTION!
//
// Returns: transformed north star y estimate of where
//          robot should now be in global coordinate system
// TODO?
float Robot::_getNSHalfTransY() {
    float result;
    int room = _robotInterface->RoomID();
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

    mMult(transform, 1, 2, coords, 2, 1, &result);

    //scale
    //result /= ROOM_SCALE[1][room-2];

    //move
    //result += ROOM_Y_SHIFT[room-2];

    return result;
}

// Returns: transformed north star theta estimate of where
//          robot should now be in global coordinate system
// TODO?
float Robot::_getNSTransTheta() {
    float result = _getNSTheta();
    int room = _robotInterface->RoomID();
    result -= (ROOM_ROTATION[room-2] * (PI/180.0));
    
    if (result < -PI) {
        result += 2*PI;
    }
    
    if (result < 0) {
        result += 2*PI;
    }

    return result;
}

// Updates transformed wheel encoder pose estimate of where
// robot should now be in global coordinate system
void Robot::_updateWEPose() {
    float lastTheta = _wePose->getTheta();
    
    float deltaX = _getWETransDeltaX();
    float deltaY = _getWETransDeltaY();
    float dTheta = _getWETransDeltaTheta();
   
    float newTheta = lastTheta + dTheta;

    if (newTheta > 2*PI) {
        newTheta -= 2*PI;
    }
    else if (newTheta < 0) {
        newTheta += 2*PI;
    }
    
    if (lastTheta > (3/2.0)*PI && newTheta < PI/2.0) {
        _passed2PIwe = !_passed2PIwe;
    }
    else if (lastTheta < PI/2.0 && newTheta > (3/2.0)*PI) {
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

    _wePose->add(deltaX, deltaY, 0);
    _wePose->setTheta(newTheta);
}
 
// Updates transformed north star pose estimate of where
// robot should now be in global coordinate system
void Robot::_updateNSPose() {
    float lastTheta = _nsPose->getTheta();

    float newX = _getNSTransX();
    float newY = _getNSTransY();
    float newTheta = _getNSTransTheta();
    
    _nsPose->setX(newX);
    _nsPose->setY(newY);
    
    if (lastTheta > (3/2.0)*PI && newTheta < PI/2.0) {
        _passed2PIns = !_passed2PIns;
    }
    else if (lastTheta < PI/2.0 && newTheta > (3/2.0)*PI) {
        _passed2PIns = !_passed2PIns;
    }
    
    if (_passed2PIns && (lastTheta > PI && newTheta < PI)) {
        _nsPose->modifyRotations(1);
        _passed2PIns = false;
    }
    else if (_passed2PIns && (lastTheta < PI && newTheta > PI)) {
        _nsPose->modifyRotations(-1);
        _passed2PIns = false;
    }

    _nsPose->setTheta(newTheta);
}
