/**
 * camera.cpp
 * 
 * @brief 
 * 		This class defines the rovio's camera. It has functions for accessing, storing, and processing the image
 * 		returned from the robot
 * 
 * @author
 * 		Shawn Hanna
 * 		Tom Nason
 * 		Joel Griffith
 * 
 * @date
 * 		created - 3//2012
 * 		modified - 3/24/2012
 **/

#include "camera.h"
#include "logger.h"
#include "utilities.h"
#include <robot_color.h>

#define MAX_SLOPE 2.5
#define MIN_SLOPE 0.01

#define RIGHT_LEFT_SLOPE -0.35
#define RIGHT_RIGHT_SLOPE -2.22

#define LEFT_LEFT_SLOPE 2.6
#define LEFT_RIGHT_SLOPE 0.55

#define MAX_SLOPE_DIFFERENCE 0.5

#define MAX_CAMERA_ERRORS 7

Camera::Camera(RobotInterface *robotInterface) {
    _robotInterface = robotInterface;
    _centerDistErrorFilter = new FIRFilter("filters/cam_center_dist_error.ffc");
    _slopeErrorFilter = new FIRFilter("filters/cam_slope_error.ffc");
    setQuality(RI_CAMERA_QUALITY_HIGH);
    setResolution(RI_CAMERA_RES_320);
    _pinkThresholded = NULL;
    _yellowThresholded = NULL;
    _pinkSquares = NULL;
    _yellowSquares = NULL;

    cvNamedWindow("Thresholded", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("Biggest Squares Distances", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("Slopes", CV_WINDOW_AUTOSIZE);

    _robotInterface->Move(RI_HEAD_MIDDLE, 1);
}

Camera::~Camera() {
    delete _centerDistErrorFilter;
    delete _slopeErrorFilter;
    delete _pinkThresholded;
    delete _yellowThresholded;
    delete _pinkSquares;
    delete _yellowSquares;
    // TODO: close windows
    _robotInterface->Move(RI_HEAD_DOWN, 1);
}

void Camera::setQuality(int quality) {
    if (_robotInterface->CameraCfg(0x7F, 
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

void Camera::drawX(IplImage *image, squares_t *square, CvScalar color) {
    if (square == NULL) {
        return;
    }
    
    CvPoint pt1, pt2;

    // Draw an X marker on the image
    int sqAmt = (int) (sqrt(square->area) / 2); 

    // Upper Left to Lower Right
    pt1.x = square->center.x - sqAmt;
    pt1.y = square->center.y - sqAmt;
    pt2.x = square->center.x + sqAmt;
    pt2.y = square->center.y + sqAmt;
    cvLine(image, pt1, pt2, color, 3, CV_AA, 0);

    // Lower Left to Upper Right
    pt1.x = square->center.x - sqAmt;
    pt1.y = square->center.y + sqAmt;
    pt2.x = square->center.x + sqAmt;
    pt2.y = square->center.y - sqAmt;
    cvLine(image, pt1, pt2, color, 3, CV_AA, 0);
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
    */

    // get a red and pink thresholded and or them together
    IplImage *redThresholded = getThresholdedImage(RED_LOW, RED_HIGH);
    while (redThresholded == NULL) {
        redThresholded = getThresholdedImage(RED_LOW, RED_HIGH);
    }
    _pinkThresholded = getThresholdedImage(PINK_LOW, PINK_HIGH);
    while (_pinkThresholded == NULL) {
        _pinkThresholded = getThresholdedImage(PINK_LOW, PINK_HIGH);
    }
    cvOr(_pinkThresholded, redThresholded, _pinkThresholded);
    
    _yellowThresholded = getThresholdedImage(YELLOW_LOW, YELLOW_HIGH);
    while (_yellowThresholded == NULL) {
        _yellowThresholded = getThresholdedImage(YELLOW_LOW, YELLOW_HIGH);
    }
    cvSmooth(_pinkThresholded, _pinkThresholded, CV_BLUR_NO_SCALE);
    cvSmooth(_yellowThresholded, _yellowThresholded, CV_BLUR_NO_SCALE);

    cvShowImage("Thresholded", _pinkThresholded);

    _pinkSquares = findSquaresOf(COLOR_PINK, DEFAULT_SQUARE_SIZE);
    _yellowSquares = findSquaresOf(COLOR_YELLOW, DEFAULT_SQUARE_SIZE);

    // update the open windows instantly
    cvWaitKey(10);
}

float Camera::centerError(int color) {
    int numGoodSlopeErrors = 0;
    int numGoodCenterDistErrors = 0;
    float totalGoodSlopeError = 0.0;
    float totalGoodCenterDistError = 0.0;

    for (int i = 0; i < MAX_CAMERA_ERRORS; i++) {
        update();

        float slopeError = corridorSlopeError(color);
        float centerDistError = centerDistanceError(color);

        if (slopeError != -999) {
            numGoodSlopeErrors++;
            totalGoodSlopeError += slopeError;
        }

        if (centerDistError != -999) {
            numGoodCenterDistErrors++;
            totalGoodCenterDistError += centerDistError;
        }
    }

    LOG.write(LOG_LOW, "centerError", 
              "avg slope error: %f", 
              (numGoodSlopeErrors == 0) ? -999 : (totalGoodSlopeError / (float)numGoodSlopeErrors));
    LOG.write(LOG_LOW, "centerError", 
              "avg center dist error: %f", 
              (numGoodCenterDistErrors == 0) ? -999 : (totalGoodCenterDistError / (float)numGoodCenterDistErrors));

    if (numGoodSlopeErrors == 0) {
        // we couldn't find any slopes, so default
        // to using the center distance instead
        printf("Couldn't find any slopes, default to using center distance...\n");
        if (numGoodCenterDistErrors == 0) {
            // we also couldn't find any squares.. oh snap
			printf("Couldn't find center distance either... OH SNAP!!!!!\n");
            return 0;
        }
        return totalGoodCenterDistError / (float)numGoodCenterDistErrors;
    }

    return totalGoodSlopeError / (float)numGoodSlopeErrors;
}

/** ***************************************
 * Corridor Slope Error Function 
 * Parameters: color flag corresponding to square color of interest
 * Returns: integer corresponding to difference in slope of flags on the corridor walls
 *
 * This function takes the image data (square locations) and performs a linear regression on the square locations,
 * in order to define a measure of centered-ness in the corridor based on perceived slope of the corridor walls.
 * ****************************************/
float Camera::corridorSlopeError(int color) {
    int error = 0;
	float scalar=.3;

    regressionLine leftSide = leastSquaresRegression(color, SIDE_LEFT);
    regressionLine rightSide = leastSquaresRegression(color, SIDE_RIGHT);
	
	LOG.write(LOG_HIGH, "leftRegLine", "%f\n", leftSide.slope);
	LOG.write(LOG_HIGH, "rightRegLine", "%f\n", rightSide.slope);

    IplImage *bgr = getBGRImage();
    if (bgr != NULL) {
        CvPoint leftStart;
        CvPoint leftEnd;
        CvPoint rightStart;
        CvPoint rightEnd;
        leftStart.x = 0;
        leftStart.y = ((float)leftSide.slope) * 0 + leftSide.intercept;
        leftEnd.x = ((float)bgr->width / 2.0);
        leftEnd.y = ((float)leftSide.slope) * ((float)bgr->width / 2.0) + leftSide.intercept;
        rightStart.x = (float)bgr->width;
        rightStart.y = ((float)rightSide.slope) * ((float)bgr->width) + rightSide.intercept;
        rightEnd.x = ((float)bgr->width / 2.0);
        rightEnd.y = ((float)rightSide.slope) * ((float)bgr->width / 2.0) + rightSide.intercept;
        cvLine(bgr, leftStart, leftEnd, RED, 3, CV_AA, 0);
        cvLine(bgr, rightStart, rightEnd, GREEN, 3, CV_AA, 0);
        cvShowImage("Slopes", bgr);
        cvReleaseImage(&bgr);
    }

    //For now, to debug/make meaningful observations about line fits
    LOG.printfScreen(LOG_HIGH, "regression", "Linear regression performed. Left squares found: %d\n", leftSide.numSquares);
    LOG.printfScreen(LOG_HIGH, "regression", "Linear regression performed. Right squares found: %d\n", rightSide.numSquares);

    LOG.printfScreen(LOG_HIGH, "regression", "Left equation: Y = %f*X + %f\n", leftSide.slope, leftSide.intercept);
    LOG.printfScreen(LOG_HIGH, "regression", "Right equation: Y = %f*X + %f\n", rightSide.slope, rightSide.intercept);

	bool hasSlopeRight = false;
    bool hasSlopeLeft = false;

    if(leftSide.numSquares >= 2 && rightSide.numSquares >= 2) { //if lines are found on both sides...
		//do something to define error relative to the differences of the slopes

        // sanity-check the slopes to make sure we have good ones to go off of
		if(rightSide.slope > RIGHT_RIGHT_SLOPE && rightSide.slope < RIGHT_LEFT_SLOPE && rightSide.slope != -0.500000 && rightSide.slope != 0.500000) {
			//seems like a good slope
			hasSlopeRight = true;
			LOG.write(LOG_HIGH, "rightRegression", "%f\n", rightSide.slope);
		}
		if(leftSide.slope < LEFT_LEFT_SLOPE && leftSide.slope > LEFT_RIGHT_SLOPE && leftSide.slope != -0.500000 && leftSide.slope != 0.500000) {
			//seems like a good slope
			hasSlopeLeft = true;
			LOG.write(LOG_HIGH, "leftRegression", "%f\n", leftSide.slope);
		}
		
		float difference = leftSide.slope + rightSide.slope;
		if (hasSlopeLeft && hasSlopeRight) {
			if(difference > MAX_SLOPE_DIFFERENCE) {
				return -1;
			} else if (difference < -MAX_SLOPE_DIFFERENCE) {
				return 1;
			} else {
				return -difference/MAX_SLOPE;
			}
		}

		if( hasSlopeLeft && !hasSlopeRight ){
			//no right slope, so interpolate based on left
			LOG.write(LOG_LOW, "corridorSlopeError", "only left slope, interpolating\n");
			float leftTranslate = Util::mapValue(leftSide.slope, LEFT_LEFT_SLOPE, LEFT_RIGHT_SLOPE, -1, 1);
            LOG.write(LOG_LOW, "corridorSlopeError", "left translate: %f\n", leftTranslate);
			return leftTranslate;
            // used to return -1
		}
		
		if( !hasSlopeLeft && hasSlopeRight ){
			//no right slope, so interpolate based on left
            LOG.write(LOG_LOW, "corridorSlopeError", "only right slope, interpolating\n");
			float rightTranslate  = Util::mapValue(rightSide.slope, RIGHT_LEFT_SLOPE, RIGHT_RIGHT_SLOPE, -1, 1);
            LOG.write(LOG_LOW, "corridorSlopeError", "right translate: %f\n", rightTranslate);
			return rightTranslate;
            // used to return 1
		}
		
		if (!hasSlopeLeft && !hasSlopeRight){
            LOG.write(LOG_LOW, "corridorSlopeError", "no slopes!\n");
			return -999.0;
		}
	}
    // we didn't have enough squares to be useful
    return -999.0;
}


/***********************
*Least Squares Regression, generic function
*Full of debugging code, not mathematically correct
*Left side always works, right side appears to have transient problems, math according to Shawn is inconsistent as well
*Let's take a look at this once we have things running better
*
************************/
regressionLine Camera::leastSquaresRegression(int color, int side) {
    
    IplImage *thresholded;
    squares_t *squares;

    int width;
    int squareCount = 0;
    float xAvg = 0;
    float yAvg = 0;
    float xSqSum = 0;
    float xySum = 0;
    
    regressionLine result;

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

    LOG.printfScreen(LOG_HIGH, "regression","center: %d\n",center);
    
    squares_t *curSquare = squares;
    while (curSquare != NULL) {
	switch (side) {
	    case SIDE_LEFT:	
        	if (curSquare->center.x < center) {	
			LOG.printfScreen(LOG_HIGH, "regression","Left square: x: %d y:%d area: %d\n",curSquare->center.x, curSquare->center.y, curSquare->area);
			squareCount++;
		}
		break;
	    case SIDE_RIGHT:
		if (curSquare->center.x > center) {
			squareCount++;
			LOG.printfScreen(LOG_HIGH, "regression","Right square: x: %d y:%d area: %d\n",curSquare->center.x, curSquare->center.y, curSquare->area);
		}
		break;
	    }
        curSquare = curSquare->next;
    }

    result.numSquares = squareCount;
    
    if(squareCount >= 2) { 

        //Linear regression algorithm
        //Ref: http://mathworld.wolfram.com/LeastSquaresFitting.html
        curSquare = squares;
        while (curSquare != NULL) {
	    switch (side) {
	        case SIDE_LEFT:	
        	    if (curSquare->center.x < center) {
	    		xAvg += curSquare->center.x;
	    		yAvg += curSquare->center.y;
	    		xSqSum += curSquare->center.x * curSquare->center.x;
	    		xySum += curSquare->center.x * curSquare->center.y;
		    } 
		    break;
	        case SIDE_RIGHT:
		    if (curSquare->center.x > center) {
	    		xAvg += curSquare->center.x;
	    		yAvg += curSquare->center.y;
	    		xSqSum += curSquare->center.x * curSquare->center.x;
	    		xySum += curSquare->center.x * curSquare->center.y;
		    } 
		    break;
	    }
	    curSquare = curSquare->next;
        }   

        xAvg /= result.numSquares;
        yAvg /= result.numSquares;

	/*float ssxx = 0;
	float ssyy = 0;
	float ssxy = 0;
        curSquare = squares;
        while (curSquare != NULL) {
	    switch (side) {
	        case SIDE_LEFT:	
        	    if (curSquare->center.x < center) {
	    		ssxx += (curSquare->center.x - xAvg) * (curSquare->center.x - xAvg);
			ssyy += (curSquare->center.y - yAvg) * (curSquare->center.y - yAvg);
			ssxy += (curSquare->center.x - xAvg) * (curSquare->center.y - yAvg);
		    } 
		    break;
	        case SIDE_RIGHT:
		    if (curSquare->center.x > center) {
	    		ssxx += (curSquare->center.x - xAvg) * (curSquare->center.x - xAvg);
			ssyy += (curSquare->center.y - yAvg) * (curSquare->center.y - yAvg);
			ssxy += (curSquare->center.x - xAvg) * (curSquare->center.y - yAvg);
		    } 
		    break;
	    }
	    curSquare = curSquare->next;
	}

	
    	LOG.printfScreen(LOG_HIGH, "regression", "%d Equation: Y = %f*X + %f\n", side, ssxy/ssxx, (yAvg-((ssxy/ssxx)*xAvg)));
	*/
        result.intercept = (((yAvg * xSqSum) - (xAvg * xySum) ) / (xSqSum - (result.numSquares * xAvg * xAvg)));
        result.slope = ((xySum - (result.numSquares * xAvg * yAvg)) / (xSqSum - (result.numSquares * xAvg * xAvg)));
    
    } else { //We don't perform an extrapolation if there aren't enough squares
        result.intercept = -999; //Some sort of error flag value, though we can also just check the numSquares value
		result.slope = -999;
    }

    return result;
}


/* Returns an error in range [-1, 1] where 0 is no error */
// negative means move right
// positive means move left
float Camera::centerDistanceError(int color) {
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
    
    IplImage *bgr = getBGRImage();
    if (bgr != NULL) {
        drawX(bgr, leftSquare, RED);
        drawX(bgr, rightSquare, GREEN);
        CvPoint lineStart;
        CvPoint lineEnd;
        lineStart.x = center;
        lineStart.y = 0;
        lineEnd.x = center;
        lineEnd.y = bgr->height;
        cvLine(bgr, lineStart, lineEnd, BLUE, 3, CV_AA, 0);
        cvShowImage("Biggest Squares Distances", bgr);
        cvReleaseImage(&bgr);
    }

    // do we have two largest squares?
    if (leftSquare != NULL && rightSquare != NULL) {
        if (!onSamePlane(leftSquare, rightSquare)) {
            // if they're not on the same plane,
            // set the square with the smallest area to
            // be the one too far back
            if (leftSquare->area > rightSquare->area) {
                return -0.25;
            }
            else {
                return 0.25;
            }
        }
    }

    if (leftSquare == NULL && rightSquare == NULL) {
    	return -999;
    } 
    else if (leftSquare == NULL) {
        // it seems to be out of view, so we set the error to 
        // the max it could be
        return -1;
    } 
    else if (rightSquare == NULL) {
        return 1;
    }

    // otherwise, we have two squares, so find the difference
    int leftError = center - leftSquare->center.x;
    int rightError = center - rightSquare->center.x;

    //Return difference in errors
    return (float)(leftError + rightError) / (float)center;
}

/* Error is defined to be the distance of the square
   from the center of the camera
 */

bool Camera::onSamePlane(squares_t *leftSquare, squares_t *rightSquare) {
    float slope = (float)(leftSquare->center.y - rightSquare->center.y) / 
                  (float)(leftSquare->center.x - rightSquare->center.x);
    return (fabs(slope) <= MAX_PLANE_SLOPE);
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

// Finds squares of a given color in the passed image
squares_t* Camera::findSquaresOf(int color, int areaThreshold) {
    squares_t *squares;
    switch (color) {
    case COLOR_PINK:
        squares = findSquares(_pinkThresholded, areaThreshold);
        break;
    case COLOR_YELLOW:
        squares = findSquares(_yellowThresholded, areaThreshold);
        break;
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
    // was CV_RETR_EXTERNAL
    cvFindContours(canny, storage, &contours, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));
            
    // Test each contour to find squares
    while(contours) {
        // Approximate a contour with accuracy proportional to the contour perimeter
        result = cvApproxPoly(contours, sizeof(CvContour), storage, CV_POLY_APPROX_DP, cvContourPerimeter(contours)*0.1, 0 );
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

    // smooth the image to make colors more uniform
    //cvSmooth(hsv, hsv, CV_BLUR_NO_SCALE);

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
