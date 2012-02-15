#include "robot.h"
#include <math.h>

#define DEFAULT_NUM_FAILS 5

#define MAX_THETA_ERROR 0.5236 // 30 degrees
// .26 // 15 degrees
#define MAX_DIST_ERROR 10.0 // in cm

Robot::Robot(std::string address, int id) {
    _robotInterface = new RobotInterface(address, id);

    _nsXFilter = new FIRFilter("filters/ns_x.ffc");
    _nsYFilter = new FIRFilter("filters/ns_y.ffc");
    _nsThetaFilter = new FIRFilter("filters/ns_theta.ffc"); // empty filter for now
    _weLeftFilter = new FIRFilter("filters/we.ffc");
    _weRightFilter = new FIRFilter("filters/we.ffc");
    _weRearFilter = new FIRFilter("filters/we.ffc");

    name=address;
	
	_passed2PIns = false;
	_passed2PIwe = false;

    setFailLimit(DEFAULT_NUM_FAILS);

    _wePose = new Pose(0, 0, 0);
    _nsPose = new Pose(0, 0, 0);
    _pose = new Pose(0, 0, 0);
	_startingNSPose = new Pose(0,0,0);

    // bind _pose to the kalman filter
    _kalmanFilter = new Kalman(_pose);

    printf("kf initialized\n");

    PIDConstants distancePIDConstants;
    PIDConstants thetaPIDConstants;

    distancePIDConstants.kp = .8;
    distancePIDConstants.ki = .05;
    distancePIDConstants.kd = .05;

    thetaPIDConstants.kp = .65;
    thetaPIDConstants.ki = .001;
    thetaPIDConstants.kd = .001;

    float maxIntGain=0.1, minIntGain=-.1, maxThetaIntGain=.05, minThetaIntGain= -.05;

    _distancePID = new PID(&distancePIDConstants, maxIntGain, minIntGain);
    _thetaPID = new PID(&thetaPIDConstants, maxThetaIntGain, minThetaIntGain);

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
	delete _startingNSPose;

    delete _distancePID;
    delete _thetaPID;
}

void Robot::prefillData(){
	int order=_nsXFilter->getOrder();
	printf("Prefilling:");
	for (int i=0; i<order+1; i++){
		printf(" %d",i);
		update();
	}
	_startingNSPose->setX(_nsPose->getX());
	_startingNSPose->setY(_nsPose->getY());
	_startingNSPose->setTheta(_nsPose->getTheta());
	printf("\n");
}

// Moves to a location in the global coordinate system (in cm) until theta error is exceeded
void Robot::moveTo(float x, float y) {
    // find current total magnitude of the error.
    // Then, if we are not going straight towards the target, we will turn

    float thetaError;

    do {
		printf("Current Nav: %d\n",getStrength());
        thetaError = moveToUntil(x, y, MAX_THETA_ERROR);
		printf("REturnedVALLUEEEEEEE: %f\n", thetaError);
		update();
        if (thetaError != 0) {
            printf("adjusting theta...\n");
            turnTo(_pose->getTheta()-thetaError, MAX_THETA_ERROR);
            printf("theta acceptable!\n");
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

    do {
		printf("GOING TO : %f, %f\n", x, y);
        update();

        yError = y - _wePose->getY();
        xError = x - _wePose->getX();
		
        thetaDesired = atan2(yError, xError);
        thetaDesired = (float)(fmod(2*PI+thetaDesired, 2*PI));
		printf("desired theta: %f\n", thetaDesired);
        /*
		if(thetaDesired<0){
			thetaDesired+=2*PI;
		}
        */
		thetaError = thetaDesired - _pose->getTheta();
		if(thetaError>PI){
			thetaError = -(2*PI-thetaError);
		}
		
        distError = sqrt(yError*yError + xError*xError);

		printf("Value of wheel encoder x: %d\t\ty: %d\t\ttheta: %f\n", (int) _wePose->getX(), (int) _wePose->getY(), _wePose->getTheta());
		printf("Value of north star    x: %d\t\ty: %d\t\ttheta: %f\n", (int) _nsPose->getX(), (int) _nsPose->getY(), _nsPose->getTheta());
		printf("Value of kalman pose   x: %d\t\ty: %d\t\ttheta: %f\n", (int) _pose->getX(), (int) _pose->getY(), _pose->getTheta());

        // TODO: remove
		printf("Xerror:\t\t\t\t\t\t\t\t\t\t\t%f\n", xError);
		printf("Yerror:\t\t\t\t\t\t\t\t\t\t\t\t\t%f\n", yError);
		printf("Theta error:\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t%f\n", thetaError);
        printf("Distance Error = %f\n", distError);
		printf("pose theta: %f\n", _pose->getTheta());
		
		printf("Theta error: %f\t\tLiMIT: %f\t\tABS of tE: %f\n", thetaError, thetaErrorLimit, fabs(thetaError));

        distGain = _distancePID->updatePID(distError);
        _thetaPID->updatePID(thetaError);

        if ((fabs(thetaError) > thetaErrorLimit) && xError < 50) {
            return thetaError;
        }

        // going relatively straight
        printf("GOING FORWARD!!!\t\t(%d)\n",(int)1.0/(distGain));
		update();
        moveForward(5);
    } while (distError > MAX_DIST_ERROR);

    return 0; // no error when we've finished
}

//theta is between -pi and pi
void Robot::turnTo(float theta, float thetaErrorLimit) {
    float thetaError;

    float thetaGain;

    do {
		update();
		
		float poseTheta=_pose->getTheta();
		float nsPose =_nsPose->getTheta();
		
		if(theta<(-thetaErrorLimit)){
			printf("Turn right, theta error < -limit \n");
			turnRight(5);
		}
		else if(theta>thetaErrorLimit){
			printf("I'm turning left, theta error > limit\n");
			turnLeft(5);
		}
		
		printf("pose theta: %f\n", poseTheta);
		printf("ns pose theta: %f\n", nsPose);
		printf("theta: %f\n", theta);
        printf("Theta error: %f\n", thetaError);
		
		printf("Value of wheel encoder x: %d\t\ty: %d\t\ttheta: %f\n", (int) _wePose->getX(), (int) _wePose->getY(), _wePose->getTheta());
		printf("Value of north star    x: %d\t\ty: %d\t\ttheta: %f\n", (int) _nsPose->getX(), (int) _nsPose->getY(), _nsPose->getTheta());
		printf("Value of kalman pose   x: %d\t\ty: %d\t\ttheta: %f\n", (int) _pose->getX(), (int) _pose->getY(), _pose->getTheta());

		break;
		
        thetaGain = _thetaPID->updatePID(thetaError);

/*
        if (error >= 2*PI){
            theta -= 2*PI;
        }
        else if(error<= -1*2*PI){
            theta += 2*PI;
        }
*/

        if(fabs(thetaError) > thetaErrorLimit) {
            if (thetaError > PI){
                turnRight(10);
            }
            else{
                turnRight(10);
            }
		}
    } while (fabs(thetaError) > thetaErrorLimit);
}

void Robot::moveForward(int speed) {
    if (!isThereABitchInMyWay()) {
        _robotInterface->Move(RI_MOVE_FORWARD, speed);
    }
    else if(name=="Optimus"){
        printf("No No No No No No No!!!\t\tDETECTION!");
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
    if (_robotInterface->IR_Detected()) {
        return true;
    }
    return false;
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
	if(newX>0.3)
		newX=.3;
	if(newY>0.3)
		newY=.3;
	if(newTheta>0.3)
		newTheta=.3;

	//_kalmanFilter->setNSUncertainty(.15, .15, .08);
	//printf("certainties: %f %f %f\n", newX, newY, newTheta);
	//update the kalman constants for WE

	if(getStrength()>13222){ // It's OVER 9000
		//reset the theta on the we
		_wePose->setTheta(_nsPose->getTheta());
		_wePose->_numRotations= (_nsPose->_numRotations);
		printf("\a");
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

// Returns: filtered wheel encoder overall delta theta
//          in terms of robot axis
// TODO: remove, unnecessary
float Robot::_getWEDeltaTheta() {
    float rearDeltaX = _getWEDeltaXRear();

	return -Util::weToCM(rearDeltaX)/(ROBOT_DIAMETER / 2.0);
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
   using namespace Util;
   float result;
   int room = _robotInterface->RoomID();
   float coords[2];
   float transform[2];

   coords[1] = _getNSX();
   coords[0] = _getNSY();
   transform[0] = cos(ROOM_ROTATION[room-2]);
   transform[1] = -sin(ROOM_ROTATION[room-2]);

   if(ROOM_FLIPX[room-2] == 1) {
    transform[1] = -transform[1];
    transform[0] = -transform[0];
   }

   mMult(transform, 1, 2, coords, 2, 1, &result);

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
   using namespace Util;
   float result;
   int room = _robotInterface->RoomID();
   float coords[2];
   float transform[2];

   coords[1] = _getNSX();
   coords[0] = _getNSY();
   transform[0] = cos(ROOM_ROTATION[room-2]);
   transform[1] = -sin(ROOM_ROTATION[room-2]);

   if(ROOM_FLIPX[room-2] == 1) {
    transform[1] = -transform[1];
    transform[0] = -transform[0];
   }

   mMult(transform, 1, 2, coords, 2, 1, &result);

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
   using namespace Util;

   float result;
   int room = _robotInterface->RoomID();
   float coords[2];
   float transform[2];

   coords[1] = _getNSX();
   coords[0] = _getNSY();

   transform[0] = sin(ROOM_ROTATION[room-2]);
   transform[1] = cos(ROOM_ROTATION[room-2]);

   if(ROOM_FLIPY[room-2] == 1) {
    transform[1] = -transform[1];
    transform[0] = -transform[0];
   }

   mMult(transform, 1, 2, coords, 2, 1, &result);



   //scale
   result /= ROOM_SCALE[1][room-2];

   //move
   result += ROOM_Y_SHIFT[room-2];

   //Correction for skew in room 2
   if(room == 2) {
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
   using namespace Util;

   float result;
   int room = _robotInterface->RoomID();
   float coords[2];
   float transform[2];

   coords[1] = _getNSX();
   coords[0] = _getNSY();

   transform[0] = sin(ROOM_ROTATION[room-2]);
   transform[1] = cos(ROOM_ROTATION[room-2]);

   if(ROOM_FLIPY[room-2] == 1) {
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
	
	//result-=_startingNSPose->getTheta();
	
    if(result < -PI) {
        result += 2 * PI;
    }
    
    if(result<0)
		result+=2*PI;

    return result;
}

// Updates transformed wheel encoder pose estimate of where
// robot should now be in global coordinate system
void Robot::_updateWEPose() {
	float lastTheta=_wePose->getTheta();
	
    float deltaX = _getWETransDeltaX();
    float deltaY = _getWETransDeltaY();
    float dTheta = _getWETransDeltaTheta();
   
	float newTheta=lastTheta+dTheta;
	float modifier = 0;
	if(newTheta>2*PI){
		newTheta=newTheta-2*PI;
	}
	else if(newTheta<0){
		newTheta+=2*PI;
	}

	//printf("NEW THETA: %f\t\tdtheta: %f\n", newTheta, dTheta);
	
	
	if((lastTheta > (3/2.0)*PI && (newTheta) < PI/2.0)) {
		_passed2PIwe = !_passed2PIwe;
	}
	else if(lastTheta < PI/2.0 && newTheta > (3/2.0)*PI){
		_passed2PIwe = !_passed2PIwe;
	}
	
	if(_passed2PIwe && (lastTheta>PI && newTheta<PI)){
		_wePose->modifyRotations(1);
		_passed2PIwe=false;
	}
	else if(_passed2PIwe && (lastTheta<PI && newTheta>PI)){
		_wePose->modifyRotations(-1);
		_passed2PIwe=false;
	}

    _wePose->add(deltaX, deltaY, (lastTheta-newTheta+modifier));
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

	//float dTheta = lastTheta-newTheta;
	
	if((lastTheta > (3/2.0)*PI && (newTheta) < PI/2.0)) {
		_passed2PIns = !_passed2PIns;
	}
	else if(lastTheta < PI/2.0 && newTheta > (3/2.0)*PI){
		_passed2PIns = !_passed2PIns;
	}
	else{
	}
	
	if(_passed2PIns && (lastTheta>PI && newTheta<PI)){
		_nsPose->modifyRotations(1);
		_passed2PIns=false;
	}
	else if(_passed2PIns && (lastTheta<PI && newTheta>PI)){
		_nsPose->modifyRotations(-1);
		_passed2PIns=false;
	}

    _nsPose->setTheta(newTheta);
}
