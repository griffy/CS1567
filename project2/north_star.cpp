#include "north_star.h"
#include "constants.h"
#include "logger.h"
#include "utilities.h"

NorthStar::NorthStar(RobotInterface *robotInterface)
: PositionSensor(robotInterface) {
	_lastRoom = -1;
	_filterX = new FIRFilter("filters/ns_x.ffc");
	_filterY = new FIRFilter("filters/ns_y.ffc");
	_filterTheta = new FIRFilter("filters/ns_theta.ffc");
}

NorthStar::~NorthStar() {
	delete _filterX;
	delete _filterY;
	delete _filterTheta;
}

/* Requires the interface be updated prior to calling */
void NorthStar::updatePose(int room) {

	if(_lastRoom != -1 && _lastRoom != room) {
		_filterX->seed(_robotInterface->X());
		_filterY->seed(_robotInterface->Y());
		_filterTheta->seed(_robotInterface->Theta());
	}

	float x = _getFilteredX();
	float y = _getFilteredY();
	float theta = _getFilteredTheta();


	LOG.printfFile(LOG_LOW, "ns_positions_raw", 
			  "%d, %f, %f, %f\n",
			  room+2, x, y, theta);
	
//	LOG.write(LOG_LOW, "ns estimates", 
//			  "filtered x: %f, filtered y: %f, filtered theta: %f",
//			  x, y, theta);

	Pose *estimate = new Pose(x, y, theta);

//Temporary
	Pose *estimate2 = new Pose(x, y, theta);
	
//	LOG.write(LOG_LOW, "ns estimates", 
//			  "pose start x: %f, pose start y: %f, pose start theta: %f, pose total theta: %f",
//			  estimate->getX(), estimate->getY(), estimate->getTheta(), estimate->getTotalTheta());

	if (room == ROOM_2) {
		// Apply specific linear transformation to Room 2, 
		// to correct for theta skew
		float xFitAngle = 0.0000204488 * x - 0.0804;
		float yFitAngle = 0.0000204488 * y - 0.0804;
		estimate->rotateEach(xFitAngle, yFitAngle, NS_ROOM_ROTATION[room]);
		estimate2->rotate(NS_ROOM_ROTATION[room]);
	}
	else {
		estimate->rotate(NS_ROOM_ROTATION[room]);
		estimate2->rotate(NS_ROOM_ROTATION[room]);
	}

	estimate->rotateEach(0, 0, THETA_SHIFT[room]);
	estimate2->rotateEach(0, 0, THETA_SHIFT[room]);

//	LOG.write(LOG_LOW, "ns estimates", 
//		  	  "room %d, pose rotate x: %f, pose rotate y: %f, pose rotate theta: %f, pose total theta: %f",
//		      room+2, estimate->getX(), estimate->getY(), estimate->getTheta(), estimate->getTotalTheta());

	float sx = NS_ROOM_SCALE[room][0];
	float sy = NS_ROOM_SCALE[room][1];
	estimate->scale(sx, sy);
	estimate2->scale(sx, sy);
//	LOG.write(LOG_LOW, "ns estimates", 
//		  	  "room %d, pose scale x: %f, pose scale y: %f, pose scale theta: %f, pose total theta: %f",
//		      room+2, estimate->getX(), estimate->getY(), estimate->getTheta(), estimate->getTotalTheta());

	float tx = COL_OFFSET[0] + NS_ROOM_ORIGINS_FROM_COL[room][0];
	float ty = COL_OFFSET[1] + NS_ROOM_ORIGINS_FROM_COL[room][1];
	estimate->translate(tx, ty);
	estimate2->translate(tx, ty);

//	LOG.write(LOG_LOW, "ns estimates", 
//		  	  "room %d, pose translate x: %f, pose translate y: %f, pose translate theta: %f, pose total theta: %f",
//		      room+2, estimate->getX(), estimate->getY(), estimate->getTheta(), estimate->getTotalTheta());


	_adjustTotalTheta(estimate->getTheta());

	_pose->setX(estimate->getX());
	_pose->setY(estimate->getY());
	_pose->setTheta(estimate->getTheta());

	LOG.printfFile(LOG_LOW, "ns_positions_clean", 
		  	  "%d, %f, %f, %f\n",
		      room+2, _pose->getX(), _pose->getY(), _pose->getTheta());

	LOG.write(LOG_LOW, "ns_estimates", 
		  	  "room %d, new pose: x: %f, y: %f, theta: %f",
		      room+2, _pose->getX(), _pose->getY(), _pose->getTheta());

	LOG.write(LOG_LOW, "ns_correction_check",
			 "%d %f %f %f %f %f %f",
		      room+2, estimate->getX(), estimate->getY(), estimate->getTheta(), estimate2->getX(), estimate2->getY(), estimate2->getTheta());

	delete estimate;
	delete estimate2;

	_lastRoom = room;
}

float NorthStar::_getFilteredX() {
    int x = _robotInterface->X();
    return _filterX->filter((float) x);
}

float NorthStar::_getFilteredY() {
    int y = _robotInterface->Y();
    return _filterY->filter((float) y);
}

float NorthStar::_getFilteredTheta() {
    float theta = _robotInterface->Theta();
    return _filterTheta->filter(theta);
}
