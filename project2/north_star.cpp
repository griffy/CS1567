#include "north_star.h"
#include "constants.h"

NorthStar::NorthStar(RobotInterface *robotInterface)
: PositionSensor(robotInterface) {
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
	float x = _getFilteredX();
	float y = _getFilteredY();
	float theta = _getFilteredTheta();

	Pose *estimate = new Pose(x, y, theta);
	if (room == ROOM_2) {
		// Apply specific linear transformation to Room 2, 
		// to correct for theta skew
		float xFitAngle = 0.0000204488 * x - 0.0804;
		float yFitAngle = 0.0000204488 * y - 0.0804;
		estimate->rotateEach(xFitAngle, yFitAngle, NS_ROOM_ROTATION[room]);
	}
	else {
		estimate->rotate(NS_ROOM_ROTATION[room]);
	}
	float sx = NS_ROOM_SCALE[room][0];
	float sy = NS_ROOM_SCALE[room][1];
	estimate->scale(sx, sy);
	float tx = COL_OFFSET[0] + NS_ROOM_ORIGINS_FROM_COL[room][0];
	float ty = COL_OFFSET[1] + NS_ROOM_ORIGINS_FROM_COL[room][1];
	estimate->translate(tx, ty);

	_adjustTotalTheta(theta);

	_pose->setX(estimate->getX());
	_pose->setY(estimate->getY());
	_pose->setTheta(estimate->getTheta());

	delete estimate;
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