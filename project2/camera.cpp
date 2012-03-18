#include "camera.h"
#include "logger.h"
#include <robot_color.h>

Camera::Camera(RobotInterface *robotInterface) {
    _robotInterface = robotInterface;
    _leftDistanceErrorFilter = new FIRFilter("filters/camera_distance_error.ffc");
    _rightDistanceErrorFilter = new FIRFilter("filters/camera_distance_error.ffc");
    setQuality(RI_CAMERA_QUALITY_HIGH);
    setResolution(RI_CAMERA_RES_640);
    _pinkThresholded = NULL;
    _yellowThresholded = NULL;
    _pinkSquares = NULL;
    _yellowSquares = NULL;
}

Camera::~Camera() {
    delete _leftDistanceErrorFilter;
    delete _rightDistanceErrorFilter;
    delete _pinkThresholded;
    delete _yellowThresholded;
    delete _pinkSquares;
    delete _yellowSquares;
}

void Camera::setQuality(int quality) {
    if (_robotInterface->CameraCfg(RI_CAMERA_DEFAULT_BRIGHTNESS, 
                        RI_CAMERA_DEFAULT_CONTRAST, 
                        5, 
                        _resolution, 
                        quality)) {
        LOG.write(LOG_HIGH, "camera settings", 
                  "Failed to change the quality to %d", quality);
    }
    else {
        _quality = quality;
    }
}

void Camera::setResolution(int resolution) {
    if (_robotInterface->CameraCfg(RI_CAMERA_DEFAULT_BRIGHTNESS, 
                        RI_CAMERA_DEFAULT_CONTRAST, 
                        5, 
                        resolution, 
                        _quality)) {
        LOG.write(LOG_HIGH, "camera settings", 
                  "Failed to change the resolution to %d", resolution);
    }
    else {
        _resolution = resolution;
    }
}

void Camera::update() {
    if (_pinkThresholded != NULL) {
        cvReleaseImage(&_pinkThresholded);
    }
    if (_yellowThresholded != NULL) {
        cvReleaseImage(&_yellowThresholded);
    }
    /*
    // FIXME: free squares memory properly
    while (_pinkSquares != NULL) {
        squares_t *square = _pinkSquares;
        if (square->next != NULL) {
            square = square->next;
        }
        delete _pinkSquares;
        _pinkSquares = square;
    }
    while (_yellowSquares != NULL) {
        squares_t *square = _yellowSquares;
        if (square->next != NULL) {
            square = square->next;
        }
        delete _yellowSquares;
        _yellowSquares = square;
    }
    */
    _pinkThresholded = getThresholdedImage(RC_PINK_LOW, RC_PINK_HIGH);
    _yellowThresholded = getThresholdedImage(RC_YELLOW_LOW, RC_YELLOW_HIGH);
    _pinkSquares = findSquaresOf(COLOR_PINK, DEFAULT_SQUARE_SIZE);
    _yellowSquares = findSquaresOf(COLOR_YELLOW, DEFAULT_SQUARE_SIZE);
}

int Camera::centerDistanceError(int color) {
	//static int rightMissCount = 0, leftMissCount = 0;

    int width;
    switch (color) {
    case COLOR_PINK:
        width = _pinkThresholded->width;
        break;
    case COLOR_YELLOW:
        width = _yellowThresholded->width;
        break;
    }

    int center = width / 2;

    squares_t *leftSquare = leftBiggestSquare(color);
    squares_t *rightSquare = rightBiggestSquare(color);

    if (leftSquare != NULL && rightSquare != NULL) {
        if (!onSamePlane(leftSquare, rightSquare)) {
            // if they're not on the same plane,
            // set the square with the smallest area to
            // be the one too far back
            if (leftSquare->area > rightSquare->area) {
                rightSquare = NULL;
            }
            else {
                leftSquare = NULL;
            }
        }
    }

    if (leftSquare == NULL) {
        // it's out of view, so we set the error to 
        // the max it could be
        return center;
    }
    else if (rightSquare == NULL) {
        return -center;
    }

    int leftError = center - leftSquare->center.x;
    int rightError = rightSquare->center.x - center;

    return leftError + rightError;

/*  
    int filteredLeftError;
    int filteredRightError;
    for (int i = 0; i < _leftDistanceErrorFilter->getOrder()+1; i++) {
        update();

        int leftError = leftSquareDistanceError(color);
        int rightError = rightSquareDistanceError(color);

        if (leftError == -1) {
            // the left square is out of view, so we
            // set the error to the max it could be
            leftMissCount++;
			leftError=_leftDistanceErrorFilter->getValue();
            //return width / 2;
        }
        else{
			leftMissCount=0;
		}
        if (rightError == -1) {
			rightMissCount++;
			rightError=_rightDistanceErrorFilter->getValue();
            //return -width / 2;
        }
        else {
			rightMissCount=0;
		}

		//arbitrary constant, could change based on testing
		filteredLeftError = (int)_leftDistanceErrorFilter->filter((float)leftError);
		if(leftMissCount>=2){
			filteredLeftError=width/2;
			if(leftMissCount==2){
				_leftDistanceErrorFilter->seed(width/2);
			}
		}
		filteredRightError = (int)_rightDistanceErrorFilter->filter((float)rightError);
		if(rightMissCount>=2){
			filteredRightError=-width/2;
			if(rightMissCount==2){
				_rightDistanceErrorFilter->seed(-width/2);
			}
		}
    }
    return filteredLeftError - filteredRightError;
*/
}

/* Error is defined to be the distance of the square
   from the center of the camera
 */

bool Camera::onSamePlane(squares_t *leftSquare, squares_t *rightSquare) {
    int difference = abs(leftSquare->center.y - rightSquare->center.y);
    return (difference <= MAX_PLANE_SLOPE);
}

squares_t* Camera::leftBiggestSquare(int color) {
    IplImage *thresholded;
    squares_t *squares;

    switch (color) {
    case COLOR_PINK:
        thresholded = _pinkThresholded;
        squares = _pinkSquares;
        break;
    case COLOR_YELLOW:
        thresholded = _yellowThresholded;
        squares = _yellowSquares;
        break;
    }

    int center = thresholded->width / 2;

    squares_t *curSquare = squares;
    squares_t *largestSquare = NULL;
    while (curSquare != NULL) {
        if (curSquare->center.x < center) {
            if (largestSquare == NULL) {
                largestSquare = curSquare;
            }
            else {
                if (curSquare->area > largestSquare->area) {
                    largestSquare = curSquare;
                }
            }
        }
        curSquare = curSquare->next;
    }

    return largestSquare;
}

squares_t* Camera::rightBiggestSquare(int color) {
    IplImage *thresholded;
    squares_t *squares;

    switch (color) {
    case COLOR_PINK:
        thresholded = _pinkThresholded;
        squares = _pinkSquares;
        break;
    case COLOR_YELLOW:
        thresholded = _yellowThresholded;
        squares = _yellowSquares;
        break;
    }

    int center = thresholded->width / 2;
    
    squares_t *curSquare = squares;
    squares_t *largestSquare = NULL;
    while (curSquare != NULL) {
        if (curSquare->center.x > center) {
            if (largestSquare == NULL) {
                largestSquare = curSquare;
            }
            else {
                if (curSquare->area > largestSquare->area) {
                    largestSquare = curSquare;
                }
            }
        }
        curSquare = curSquare->next;
    }

    return largestSquare;
}

/**
  * Calculates the slope and y intercept of
  * the given squares, based on their X and Y pixel centers of mass
  *
  * Also gives an r2 calculation, which shows how accuratly the line
  * represents the points, with a value of 0 meaning none, to 1 meaning
  * the line hits all the points
  * 
  * @arg line - pass in a pointer to the lineStruct that will receive the value for the line
**/

void Camera::calculateSlope(squares_t *squares, lineStruct *line){
	squares_t *currSqr=(squares_t*) squares;
	float xAvg, yAvg;
	float sumx = 0, sumy=0, sumxy=0;
	int count=0;
	// calculate average x and y values
	while(currSqr != NULL){
		sumx+=currSqr->center.x;
		sumy+=currSqr->center.y;
		count++;
		currSqr=currSqr->next;
	}
	xAvg = sumx/count;
	yAvg = sumy/count;

	if(count <= 1){
		// either 1 or 0 squares, cannot find a line with one point

		line->slope = -999.0;
		line->yInt = -999.0;
		line->r2 = -999.0;
		return ;
	}

	//calculate Sx, Sy, Sxy
	sumx=0; sumy=0;
	currSqr=(squares_t*) squares;
	while(currSqr != NULL){
		sumx+=(currSqr->center.x - xAvg)*(currSqr->center.x - xAvg);
		sumy+=(currSqr->center.y - yAvg)*(currSqr->center.y - yAvg);
		sumxy+=(currSqr->center.x - xAvg)*(currSqr->center.y - yAvg);
		currSqr=currSqr->next;
	}

	line->slope = sumxy/sumx;
	line->yInt = yAvg - line->slope*xAvg;
	line->r2 = sumxy/sqrt(sumx*sumy);
}

/**
 * @return the location of the robot in the grid based on a scale of 0..1
 * 		with 0 being farthest left in the grid, and 1 being farthest right
 */
float Camera::estimatePos (squares_t *leftSquares, squares_t *rightSquares){
	//find the lines for both squares
	lineStruct leftLine;
	lineStruct rightLine;
	
	calculateSlope(leftSquares, &leftLine);
	calculateSlope(rightSquares, &rightLine);
	
	//check if slopes could not be calculated
	if (leftLine.slope == -999.0){
		LOG.write(LOG_HIGH, "camera image", "Error while calculating slope of left line... cannot estimate position accurately\n");
	}
	if (rightLine.slope == -999.0){
		LOG.write(LOG_HIGH, "camera image", "Error while calculating slope of right line... cannot estimate position accurately\n");
	}
	
	LOG.write(LOG_HIGH, "camera image", "Left line: y = %f*x + %f\t\tr^2 = %f\n", leftLine.slope, leftLine.yInt, leftLine.r2);
	LOG.write(LOG_HIGH, "camera image", "Right line: y = %f*x + %f\t\tr^2 = %f\n", rightLine.slope, rightLine.yInt, rightLine.r2);
	
	return 0;
}

float Camera::findPos (squares_t *squares){
	
    int center = _pinkThresholded->width / 2;
	
	
	squares_t *rightSquares=NULL;
	squares_t *indexRS=NULL;
	
	squares_t *leftSquares = NULL;
	squares_t *indexLS = NULL;
	
    squares_t *curSquare = squares;
    while (curSquare != NULL) {
        if (curSquare->center.x > center) {
			if(rightSquares == NULL){
				rightSquares = new squares_t;
				indexRS = rightSquares;
				rightSquares->next = NULL;
			}
			else{
				indexRS->next = new squares_t;
				indexRS = indexRS->next;
				indexRS = curSquare;
				indexRS->next = NULL;
			}
        }
        else {
			if(leftSquares == NULL){
				leftSquares = new squares_t;
				indexLS = leftSquares;
				leftSquares->next = NULL;
			}
			else{
				indexLS->next = new squares_t;
				indexLS = indexLS->next;
				indexLS = curSquare;
				indexLS->next = NULL;
			}
		}
        curSquare = curSquare->next;
    }
    
    
    float result = estimatePos(leftSquares, rightSquares);
	
	//clean up old arrays
	indexRS = rightSquares;
	while (indexRS != NULL){
		squares_t *nextPointer = indexRS->next;
		delete indexRS;
		indexRS = nextPointer;
	}
	indexLS = leftSquares;
	while (indexLS != NULL){
		squares_t *nextPointer = indexLS->next;
		delete indexRS;
		indexLS = nextPointer;
	}

	return result;
}

// Finds squares of a given color in the passed image
squares_t* Camera::findSquaresOf(int color, int areaThreshold) {
    squares_t *squares;
    switch (color) {
    case COLOR_PINK:
        squares = findSquares(_pinkThresholded, areaThreshold);
        break;
    case COLOR_YELLOW:
        squares = findSquares(_yellowThresholded, areaThreshold);
    }
    return squares;
}

squares_t* Camera::findSquares(IplImage *img, int areaThreshold) {
    CvSeq* contours;
    CvMemStorage *storage;
    int i, j, area;
    CvPoint ul, lr, pt, centroid;
    CvSize sz = cvSize( img->width, img->height);
    IplImage * canny = cvCreateImage(sz, 8, 1);
    squares_t *sq_head, *sq, *sq_last;
        CvSeqReader reader;
    
    // Create storage
    storage = cvCreateMemStorage(0);
    
    // Pyramid images for blurring the result
    IplImage* pyr = cvCreateImage(cvSize(sz.width/2, sz.height/2), 8, 1);
    IplImage* pyr2 = cvCreateImage(cvSize(sz.width/4, sz.height/4), 8, 1);

    CvSeq* result;
    double s, t;

    // Create an empty sequence that will contain the square's vertices
    CvSeq* squares = cvCreateSeq(0, sizeof(CvSeq), sizeof(CvPoint), storage);
    
    // Select the maximum ROI in the image with the width and height divisible by 2
    cvSetImageROI(img, cvRect(0, 0, sz.width, sz.height));
    
    // Down and up scale the image to reduce noise
    cvPyrDown( img, pyr, CV_GAUSSIAN_5x5 );
    cvPyrDown( pyr, pyr2, CV_GAUSSIAN_5x5 );
    cvPyrUp( pyr2, pyr, CV_GAUSSIAN_5x5 );
    cvPyrUp( pyr, img, CV_GAUSSIAN_5x5 );

    // Apply the canny edge detector and set the lower to 0 (which forces edges merging) 
    cvCanny(img, canny, 0, 50, 3);
        
    // Dilate canny output to remove potential holes between edge segments 
    cvDilate(canny, canny, 0, 2);
        
    // Find the contours and store them all as a list
    cvFindContours(canny, storage, &contours, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));
            
    // Test each contour to find squares
    while(contours) {
        // Approximate a contour with accuracy proportional to the contour perimeter
        result = cvApproxPoly(contours, sizeof(CvContour), storage, CV_POLY_APPROX_DP, cvContourPerimeter(contours)*0.10, 0 );
                // Square contours should have
        //  * 4 vertices after approximation
        //  * Relatively large area (to filter out noisy contours)
        //  * Ne convex.
        // Note: absolute value of an area is used because
        // area may be positive or negative - in accordance with the
        // contour orientation
        if(result->total == 4 && fabs(cvContourArea(result,CV_WHOLE_SEQ,0)) > areaThreshold && cvCheckContourConvexity(result)) {
            s=0;
            for(i=0; i<5; i++) {
                            // Find the minimum angle between joint edges (maximum of cosine)
                if(i >= 2) {
                    t = fabs(ri_angle((CvPoint*)cvGetSeqElem(result, i),(CvPoint*)cvGetSeqElem(result, i-2),(CvPoint*)cvGetSeqElem( result, i-1 )));
                    s = s > t ? s : t;
                }
            }
                
            // If cosines of all angles are small (all angles are ~90 degree) then write the vertices to the sequence 
            if( s < 0.2 ) {
                for( i = 0; i < 4; i++ ) {
                    cvSeqPush(squares, (CvPoint*)cvGetSeqElem(result, i));
                }
            }
        }    
        // Get the next contour
        contours = contours->h_next;
    }

        // initialize reader of the sequence
    cvStartReadSeq(squares, &reader, 0);
    sq_head = NULL; sq_last = NULL; sq = NULL;
    // Now, we have a list of contours that are squares, find the centroids and area
    for(i=0; i<squares->total; i+=4) {
        // Find the upper left and lower right coordinates
        ul.x = 1000; ul.y = 1000; lr.x = 0; lr.y = 0;
        for(j=0; j<4; j++) {
            CV_READ_SEQ_ELEM(pt, reader);
            // Upper Left
            if(pt.x < ul.x)
                ul.x = pt.x;
            if(pt.y < ul.y)
                ul.y = pt.y;
            // Lower right
            if(pt.x > lr.x)
                lr.x = pt.x;
            if(pt.y > lr.y)
                lr.y = pt.y;
        }

        // Find the centroid
        centroid.x = ((lr.x - ul.x) / 2) + ul.x;
        centroid.y = ((lr.y - ul.y) / 2) + ul.y;

        // Find the area
        area = (lr.x - ul.x) * (lr.y - ul.y);

        // Add it to the storage
        sq = new squares_t;
        // Fill in the data
        sq->area = area;
        sq->center.x = centroid.x;
        sq->center.y = centroid.y;
        sq->next = NULL;
        if(sq_last == NULL) 
            sq_head = sq;   
        else 
            sq_last->next = sq;
        sq_last = sq;
    }   
    
    // Release the temporary images and data
    cvReleaseImage(&canny);
    cvReleaseImage(&pyr);
    cvReleaseImage(&pyr2);
    cvReleaseMemStorage(&storage);
    return sq_head;
}

IplImage* Camera::getHSVImage() {
    // get an image (bgr) from the camera
    IplImage *bgr = getBGRImage();
    if (bgr == NULL) {
        return NULL;
    }

	IplImage *hsv = cvCreateImage(cvGetSize(bgr), IPL_DEPTH_8U, 3);
    // convert the image from BGR to HSV
    cvCvtColor(bgr, hsv, CV_BGR2HSV);
    // free the bgr image
    cvReleaseImage(&bgr);

    return hsv;
}

IplImage* Camera::getThresholdedImage(CvScalar low, CvScalar high) {
    IplImage *hsv = getHSVImage();
    if (hsv == NULL) {
        return NULL;
    }

    IplImage *thresholded = cvCreateImage(cvGetSize(hsv), IPL_DEPTH_8U, 1);
    // pick out only the color specified by its ranges
    cvInRangeS(hsv, low, high, thresholded);
    // free the hsv image
    cvReleaseImage(&hsv);

    return thresholded;
}

IplImage* Camera::getBGRImage() {
    CvSize size;
    switch (_resolution) {
    case RI_CAMERA_RES_640:
        size = cvSize(640, 480);
        break;
    case RI_CAMERA_RES_352:
        size = cvSize(352, 240);
        break;
    case RI_CAMERA_RES_320:
        size = cvSize(320, 240);
        break;
    case RI_CAMERA_RES_176:
        size = cvSize(176, 144);
        break;
    }
    IplImage *bgr = cvCreateImage(size, IPL_DEPTH_8U, 3);

    if (_robotInterface->getImage(bgr) != RI_RESP_SUCCESS) {
        LOG.write(LOG_HIGH, "camera image", 
                  "Unable to get an image!");
        bgr = NULL;
    }
    return bgr;
}
