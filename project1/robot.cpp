#include "robot.h"
#include "utilities.h"

#define DEFAULT_NUM_FAILS 5

Robot::Robot(std::string address, int id) {
    _robotInterface = new RobotInterface(address, id);

    _nsXFilter = new FIRFilter("filters/ns_x.ffc");
    _nsYFilter = new FIRFilter("filters/ns_y.ffc");
    _nsThetaFilter = new FIRFilter("filters/ns_theta.ffc");
    _weLeftFilter = new FIRFilter("filters/we.ffc");
    _weRightFilter = new FIRFilter("filters/we.ffc");
    _weRearFilter = new FIRFilter("filters/we.ffc");

    setFailLimit(DEFAULT_NUM_FAILS);

	currPose = new Pose(0,0,0);
	
	
	
	PIDConstants distancePIDConstants;
	PIDConstants thetaPIDConstants;
	
	distancePIDConstants.ki=.001;
	distancePIDConstants.kp=1;
	distancePIDConstants.kd=1;
	
	thetaPIDConstants.ki=.001;
	thetaPIDConstants.kp=1;
	thetaPIDConstants.kd=1;

	distancePID=new PID(&distancePIDConstants);
	thetaPID=new PID(&thetaPIDConstants);

}

Robot::~Robot() {
    delete _robotInterface;
    delete _nsXFilter;
    delete _nsYFilter;
    delete _nsThetaFilter;
    delete _weLeftFilter;
    delete _weRightFilter;
    delete _weRearFilter;
	delete currPose;
}

void Robot::moveTo(int x, int y) {
	//find current total magnitude of the error.  Then, if we are not going straight towards the target, we will turn
	
	float yError=y-currPose->getY();
	float xError=x-currPose->getX();

	float error=sqrt(pow(yError,2.0)+pow(xError,2.0));
	printf("Error = %f\n",error);
	
	float distGain = distancePID->updatePID(error);
	
	float thetaError=atan(yError/xError);
	printf("Theta Error = %f\n",thetaError);
	
	float thetaGain = thetaPID->updatePID(thetaError);

	
	if(abs(thetaError)>0.1){
		//don't move, just turn
		bool turnRight=true;
		if(thetaError>0){
			turnRight=false;
			_robotInterface->Move(RI_TURN_RIGHT,5);
		}
		else{
			_robotInterface->Move(RI_TURN_LEFT,5);
		}
	}
	else{
		// going relatively straight
		_robotInterface->Move(RI_MOVE_FORWARD, distGain*10);
	}
}

void Robot::turnTo(int theta) {
	
}

void Robot::setFailLimit(int limit) {
    _failLimit = limit;
}

int Robot::getFailLimit() {
    return _failLimit;
}

// Returns: the new robot pose in terms of the global coordinate system
//          as the best estimate of its position
// TODO
Pose* Robot::getPose() {
    return new Pose(0, 0, 0);
}

// Attempts to update the robot
// Returns: true if update succeeded
//          false if update fail limit was reached
bool Robot::_update() {
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
    deltaX *= cos(DEGREE_150);
    return deltaX;
}

// Returns: filtered wheel encoder delta y for left wheel in ticks 
//          in terms of robot axis
float Robot::_getWEDeltaYLeft() {
    float deltaY = _getWEDeltaLeft();
    deltaY *= sin(DEGREE_150);
    return deltaY;
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
    deltaY *= sin(DEGREE_30);
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
    float rearDeltaX = _getWEDeltaXRear();
    // return the average
    return (leftDeltaX + rightDeltaX + rearDeltaX) / 3;
}

// Returns: filtered wheel encoder overall delta y in ticks
//          in terms of robot axis
float Robot::_getWEDeltaY() {
    float leftDeltaY = _getWEDeltaYLeft();
    float rightDeltaY = _getWEDeltaYRight();
    // return the average
    return (leftDeltaY + rightDeltaY) / 2;
}

// Returns: filtered wheel encoder overall delta theta
//          in terms of robot axis
float Robot::_getWEDeltaTheta() {
    float rear = _getWEDeltaRear();
    // FIXME: does this given formula really work?
    return rear / (PI * Util::cmToWE(ROBOT_DIAMETER));
}

// Returns: transformed wheel encoder x estimate in cm of where
//          robot should now be in global coordinate system
float Robot::_getWETransX() {
    float deltaX = _getWEDeltaX();
    float scaledDeltaX = Util::weToCM(deltaX);
    // TODO: finish
}

// Returns: transformed wheel encoder y estimate in cm of where
//          robot should now be in global coordinate system
float Robot::_getWETransY() {
    float deltaY = _getWEDeltaY();
    float scaledDeltaY = Util::weToCM(deltaY);
    // TODO: finish
}

// Returns: transformed wheel encoder theta estimate of where
//          robot should now be in global coordinate system
float Robot::_getWETransTheta() {
    float deltaTheta = _getWEDeltaTheta();
    // TODO: finish
    return deltaTheta;
}

// Returns: transformed north star x estimate of where
//          robot should now be in global coordinate system
// TODO
float Robot::_getNSTransX() {
    
}

// Returns: transformed north star y estimate of where
//          robot should now be in global coordinate system
// TODO
float Robot::_getNSTransY() {
    
}

// Returns: transformed north star theta estimate of where
//          robot should now be in global coordinate system
// TODO
float Robot::_getNSTransTheta() {
    
}

// Returns: transformed wheel encoder pose estimate of where
//          robot should now be in global coordinate system
// TODO
Pose* Robot::_getWEPose() {
    
}

// Returns: transformed north star pose estimate of where
//          robot should now be in global coordinate system
// TODO
Pose* Robot::_getNSPose() {
    
}