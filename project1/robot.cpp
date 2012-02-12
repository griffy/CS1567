#include "robot.h"

#define DEFAULT_NUM_FAILS 5

#define THETA_ERROR_MIN 0.13

Robot::Robot(std::string address, int id) {
    _robotInterface = new RobotInterface(address, id);

    _nsXFilter = new FIRFilter("filters/ns_x.ffc");
    _nsYFilter = new FIRFilter("filters/ns_y.ffc");
    _nsThetaFilter = new FIRFilter("filters/ns_theta.ffc");
    _weLeftFilter = new FIRFilter("filters/we.ffc");
    _weRightFilter = new FIRFilter("filters/we.ffc");
    _weRearFilter = new FIRFilter("filters/we.ffc");

	name=address;

    setFailLimit(DEFAULT_NUM_FAILS);

    _wePose = new Pose(0, 0, 0);
    _nsPose = new Pose(0, 0, 0);
    _pose = new Pose(0, 0, 0);

    // bind _pose to the kalman filter
    _kalmanFilter = new Kalman(_pose);

    printf("kf initialized\n");

	PIDConstants distancePIDConstants;
	PIDConstants thetaPIDConstants;

	distancePIDConstants.kp = .9;
	distancePIDConstants.ki = .05;
	distancePIDConstants.kd = .05;
	
	thetaPIDConstants.kp = .9;
	thetaPIDConstants.ki = .05;
	thetaPIDConstants.kd = .05;
	
	float maxIntGain=0.2, minIntGain=-.2, maxThetaIntGain=.2, minThetaIntGain= -.2;

	_distancePID = new PID(&distancePIDConstants, maxIntGain, minIntGain);
	_thetaPID = new PID(&thetaPIDConstants, maxThetaIntGain, minThetaIntGain);

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

// Moves to a location in the global coordinate system (in cm)
bool Robot::moveToFull(int x, int y) {
	// find current total magnitude of the error.  
    // Then, if we are not going straight towards the target, we will turn

    update();

    float yError = y - _pose->getY();
    float xError = x - _pose->getX();

    float error = sqrt(yError*yError + xError*xError);

    float distGain = _distancePID->updatePID(error);

    float thetaError = atan(yError/xError);

	

    do {
		update();

		yError = y - _pose->getY();
		xError = x - _pose->getX();
		thetaError = atan(yError/xError);
		error = sqrt(yError*yError + xError*xError);
		
		distGain = _distancePID->updatePID(error);
		
        if (abs(thetaError) > THETA_ERROR_MIN) {
			turnTo((_pose->getTheta()-thetaError));
        }
        else{
			printf("X global: %f\t\tY global: %f\t\tTheta global: %f\n",
				_pose->getX(),
				_pose->getY(),
				_pose->getTheta());

			yError = y - _pose->getY();
			xError = x - _pose->getX();

			error = sqrt(yError*yError + xError*xError);
			printf("Distance Error = %f\n", error);

            // going relatively straight
            moveForward((int)1.0/(distGain));
        }
    } while (error > 10);

    _distancePID->flushPID();
    _thetaPID->flushPID();

	return true;
}
// Moves to a location in the global coordinate system (in cm)
void Robot::moveTo(int x, int y) {
	// find current total magnitude of the error.  
    // Then, if we are not going straight towards the target, we will turn

	update();

    float yError = y - _pose->getY();
    float xError = x - _pose->getX();

    float error = sqrt(yError*yError + xError*xError);

    float distGain = _distancePID->updatePID(error);

    float thetaError = atan(yError/xError);

	if(abs(thetaError) > THETA_ERROR_MIN) {
		turnTo((_pose->getTheta()-thetaError));
	}
	else{
		printf("X global: %f\t\tY global: %f\t\tTheta global: %f\n",
			_pose->getX(),
			_pose->getY(),
			_pose->getTheta());

		yError = y - _pose->getY();
		xError = x - _pose->getX();

		error = sqrt(yError*yError + xError*xError);
		printf("Distance Error = %f\n", error);

		// going relatively straight
		moveForward((int)1.0/(distGain));
	}

    _distancePID->flushPID();
    _thetaPID->flushPID();
}

/// returns the error (in radians) of theta
float Robot::turnTo(int theta) {
	update();

    float error = theta - _pose->getTheta();
	printf("Theta Error = %f\n", error);
	
    float thetaGain = _thetaPID->updatePID(error);
	
	if(error >= 2*PI){
		theta -= 2*PI;
	}
	else if(error<= -1*2*PI){
		theta += 2*PI;
	}

	if(abs(error) > THETA_ERROR_MIN) {
		//don't move, just turn
		if (error > 0){
			_robotInterface->Move(RI_TURN_RIGHT, 5);
		}
		else{
			_robotInterface->Move(RI_TURN_LEFT, 5);
		}
	}
	return error;
}

void Robot::moveForward(int speed) {
    if (!_robotInterface->IR_Detected()) {
		if(name=="Optimus"){
			printf("No No No No No No No!!!\t\tDETECTION!");
		}
        _robotInterface->Move(RI_MOVE_FORWARD, speed);
    }
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
    // pass updated poses to kalman filter and update main pose
    _kalmanFilter->filter(_nsPose, _wePose);
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
    
    //Don't filter the theta value, weighted average is bad
    //return _nsThetaFilter->filter(theta);
    
    return theta;
}

// Returns: filtered wheel encoder delta x for left wheel in ticks 
//          in terms of robot axis
float Robot::_getWEDeltaXLeft() {
    float deltaX = _getWEDeltaLeft();
    deltaX *= cos(DEGREE_30);
    return deltaX;
}

// Returns: filtered wheel encoder delta y for left wheel in ticks 
//          in terms of robot axis
float Robot::_getWEDeltaYLeft() {
    float deltaY = _getWEDeltaLeft();
    deltaY *= cos(DEGREE_60);
    return -deltaY;
}

// Returns: filtered wheel encoder delta x for right wheel in ticks 
//          in terms of robot axis
float Robot::_getWEDeltaXRight() {
    float deltaX = _getWEDeltaRight();
    deltaX *= cos(DEGREE_30);
    return deltaX;
}

// Returns: filtered wheel encoder delta y for right wheel in ticks 
//          in terms of robot axis
float Robot::_getWEDeltaYRight() {
    float deltaY = _getWEDeltaRight();
    deltaY *= cos(DEGREE_60);
    return deltaY;
}

// Returns: filtered wheel encoder delta x for rear wheel in ticks
//          in terms of robot axis
float Robot::_getWEDeltaXRear() {
    return 0;
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
	float weOld=_getWEDeltaRear();

	int w2=rightDeltaX;
	int w1=leftDeltaX;

	float dx=(w2*cos(DEGREE_30) - w1*cos(DEGREE_30))*cos(_wePose->getTheta());


    // return the average
    return dx;
}

// Returns: filtered wheel encoder overall delta y in ticks
//          in terms of robot axis
float Robot::_getWEDeltaY() {
    float leftDeltaX = _getWEDeltaXLeft();
    float rightDeltaX = _getWEDeltaXRight();
	float weOld=_getWEDeltaRear();

	int w2=rightDeltaX;
	int w1=leftDeltaX;

	float dy=(w2*sin(DEGREE_30) - w1*sin(DEGREE_30))*sin(_wePose->getTheta());
    // return the average
    return dy;
}

// Returns: filtered wheel encoder overall delta theta
//          in terms of robot axis
float Robot::_getWEDeltaTheta() {
//    float rear = _getWEDeltaRear();
	float thetaWheelLeft=(_getWEDeltaLeft());
	float thetaWheelRight=(_getWEDeltaRight());
	float thetaWheelRear=(_getWEDeltaRear());
	
	float thetaNew=((-thetaWheelLeft+thetaWheelRight)+thetaWheelRear)/(PI*Util::cmToWE(ROBOT_DIAMETER));
    // TODO: verify that this works
    //return rear / (PI * Util::cmToWE(ROBOT_DIAMETER));
	return thetaNew;
}

// Returns: transformed wheel encoder x estimate in cm of where
//          robot should now be in global coordinate system
float Robot::_getWETransDeltaX() {
    float deltaX = _getWEDeltaX();
    float scaledDeltaX = Util::weToCM(deltaX);
    // TODO: finish
}

// Returns: transformed wheel encoder y estimate in cm of where
//          robot should now be in global coordinate system
float Robot::_getWETransDeltaY() {
    float deltaY = _getWEDeltaY();
    float scaledDeltaY = Util::weToCM(deltaY);
    // TODO: finish
}

// Returns: transformed wheel encoder theta estimate of where
//          robot should now be in global coordinate system
float Robot::_getWETransDeltaTheta() {
    float deltaTheta = _getWEDeltaTheta();
    // TODO: finish
    return deltaTheta;
}

// Returns: transformed north star x estimate of where
//          robot should now be in global coordinate system
// TODO?
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

   //move

   return result; 
}

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

   //move

   //printf("got Y: %f\n", result);
   return result; 
}

// Returns: transformed north star theta estimate of where
//          robot should now be in global coordinate system
// TODO?
float Robot::_getNSTransTheta() {
    float result = _getNSTheta();
    int room = _robotInterface->RoomID();
    result -= (ROOM_ROTATION[room-2] * (PI/180.0));
    if(result < -PI) {
    	result += 2 * PI;
    }
    return result;
}

// Updates transformed wheel encoder pose estimate of where
// robot should now be in global coordinate system
void Robot::_updateWEPose() {
    float deltaX = _getWETransDeltaX();
    float deltaY = _getWETransDeltaY();
    float deltaTheta = _getWETransDeltaTheta();
    _wePose->add(deltaX, deltaY, deltaTheta);
}

// Updates transformed north star pose estimate of where
// robot should now be in global coordinate system
void Robot::_updateNSPose() {
    float newX = _getNSTransX();
    float newY = _getNSTransY();
    float newTheta = _getNSTransTheta();
    _nsPose->setX(newX);
    _nsPose->setY(newY);
    _nsPose->setTheta(newTheta);
}
