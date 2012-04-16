/**
 * camera.cpp
 * 
 * @brief 
 *      This class defines the rovio's camera. It has functions for accessing, storing, 
 *      and processing the images returned from the camera.
 *
 * @author
 *      Shawn Hanna
 *      Tom Nason
 *      Joel Griffith
 *
 **/

#include "camera.h"
#include "logger.h"
#include "utilities.h"
#include <math.h>

Camera::Camera(RobotInterface *robotInterface) {
    _robotInterface = robotInterface;
    _pinkThresholded = NULL;
    _yellowThresholded = NULL;
    _pinkSquares = NULL;
    _yellowSquares = NULL;
    setQuality(CAMERA_QUALITY);
    setResolution(CAMERA_RESOLUTION);

    // create 3 windows that will be used to display
    // what is happening during processing of images
    cvNamedWindow("Thresholded", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("Biggest Squares Distances", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("Slopes", CV_WINDOW_AUTOSIZE);
}

Camera::~Camera() {
    if (_pinkThresholded != NULL)
        cvReleaseImage(&_pinkThresholded);
    if (_yellowThresholded != NULL)
        cvReleaseImage(&_yellowThresholded);
    if (_pinkSquares != NULL)
        delete _pinkSquares;
    if (_yellowSquares != NULL)
        delete _yellowSquares;
    // TODO: close windows
    // place the head back down since the camera is no longer being used
    _robotInterface->Move(RI_HEAD_DOWN, 1);
}

/**************************************
 * Definition: Attempts to set the rovio's camera quality. If it fails,
 *             the quality is not set and failure is logged.
 *
 * Parameters: The expected camera quality as an integer
 *
 * 
 **************************************/
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

/**************************************
 * Definition: Attempts to set the rovio's camera resolution. If it fails,
 *             the quality is not set and failure is logged.
 *
 * Parameters: The expected camera resolution as an integer
 *
 * 
 **************************************/
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

/**************************************
 * Definition: Draws an x over a square on an image
 *
 * Parameters: The image to have the newly-drawn x, the square
 *             used for creating the x, and a scalar of color for the x
 * 
 **************************************/
void Camera::markSquare(IplImage *image, squares_t *square, CvScalar color) {
    if (square == NULL || image == NULL) {
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

/**************************************
 * Definition: Retrieves new images from the camera, thresholds them, 
 *             processes them finding their squares, and updates the
 *             3 open windows
 * 
 **************************************/
void Camera::update() {
    // release the old thresholded images
    if (_pinkThresholded != NULL) {
        cvReleaseImage(&_pinkThresholded);
    }
    if (_yellowThresholded != NULL) {
        cvReleaseImage(&_yellowThresholded);
    }

    // TODO: free the old squares memory

    // get a red and pink thresholded image and or them together to 
    // have an improved pink thresholded image
    IplImage *redThresholded = getThresholdedImage(RED_LOW, RED_HIGH);
    while (redThresholded == NULL) {
        redThresholded = getThresholdedImage(RED_LOW, RED_HIGH);
    }
    _pinkThresholded = getThresholdedImage(PINK_LOW, PINK_HIGH);
    while (_pinkThresholded == NULL) {
        _pinkThresholded = getThresholdedImage(PINK_LOW, PINK_HIGH);
    }
    cvOr(_pinkThresholded, redThresholded, _pinkThresholded);
    
    // get a yellow thresholded image
    _yellowThresholded = getThresholdedImage(YELLOW_LOW, YELLOW_HIGH);
    while (_yellowThresholded == NULL) {
        _yellowThresholded = getThresholdedImage(YELLOW_LOW, YELLOW_HIGH);
    }
    // smooth both thresholded images to create more solid, blobby contours
    cvSmooth(_pinkThresholded, _pinkThresholded, CV_BLUR_NO_SCALE);
    cvSmooth(_yellowThresholded, _yellowThresholded, CV_BLUR_NO_SCALE);

    // find all squares of a given color in each thresholded image
    _pinkSquares = findSquaresOf(COLOR_PINK, DEFAULT_SQUARE_SIZE);
    _yellowSquares = findSquaresOf(COLOR_YELLOW, DEFAULT_SQUARE_SIZE);

    // show the pink thresholded image so we can see what it sees
    cvShowImage("Thresholded", _pinkThresholded);

    // update all open windows
    cvWaitKey(10);
}

float Camera::centerError(int color, int prevTagState, bool *turn) {
    *turn = false;

    int slopeTurnCount = 0;
    int centerDistTurnCount = 0;
    int intersectTurnCount = 0;

    int numGoodSlopeErrors = 0;
    int numGoodCenterDistErrors = 0;
    int numGoodIntersectErrors = 0;

    float totalGoodSlopeError = 0.0;
    float totalGoodCenterDistError = 0.0;
    float totalGoodIntersectError = 0.0;

    float totalGoodSlopeCertainty = 0.0;
    float totalGoodCenterDistCertainty = 0.0;
    float totalGoodIntersectCertainty = 0.0;

    float avgSlopeError = 0.0;
    float avgCenterDistError = 0.0;
    float avgIntersectError = 0.0;

    float avgSlopeCertainty = 0.0;
    float avgCenterDistCertainty = 0.0;
    float avgIntersectCertainty = 0.0;

    bool avgSlopeTurn = false;
    bool avgCenterDistTurn = false;
    bool avgIntersectTurn = false;

    // calculate slope and center distance errors the specified number
    // of times, ignoring -999's (which say they found nothing good)
    for (int i = 0; i < NUM_CAMERA_ERRORS; i++) {
        update();

        bool slopeTurn = false;
        bool centerDistTurn = false;
        bool intersectTurn = false;

        float slopeCertainty = 0.0;
        float centerDistCertainty = 0.0;
        float intersectCertainty = 0.0;

        float slopeError = corridorSlopeError(color, &slopeTurn, &slopeCertainty);
        float centerDistError = centerDistanceError(color, &centerDistTurn, &centerDistCertainty);
        float intersectError = intersectError(color, &intersectTurn, &intersectCertainty);

        if (slopeCertainty != 0.0) {
            numGoodSlopeErrors++;
            totalGoodSlopeError += slopeError;
            totalGoodSlopeCertainty += slopeCertainty;
            if (slopeTurn) {
                slopeTurnCount++;
            }
        }

        if (centerDistCertainty != 0.0) {
            numGoodCenterDistErrors++;
            totalGoodCenterDistError += centerDistError;
            totalGoodCenterDistCertainty += centerDistCertainty;
            if (centerDistTurn) {
                centerDistTurnCount++;
            }
        }

        if (intersectCertainty != 0.0) {
            numGoodIntersectErrors++;
            totalGoodIntersectError += intersectError;
            totalGoodIntersectCertainty += intersectCertainty;
            if (intersectTurn) {
                intersectTurnCount++;
            }
        }
    }

    float avgSlopeError = totalGoodSlopeError / (float)numGoodSlopeErrors;
    float avgCenterDistError = totalGoodCenterDistError / (float)numGoodCenterDistErrors;
    float avgIntersectError = totalGoodIntersectError / (float)numGoodIntersectErrors;

    float avgSlopeCertainty = totalGoodSlopeCertainty / (float)numGoodSlopeErrors;
    float avgCenterDistCertainty = totalGoodCenterDistCertainty / (float)numGoodCenterDistErrors;
    float avgIntersectCertainty = totalGoodIntersectCertainty / (float)numGoodIntersectErrors;

    LOG.write(LOG_LOW, "centerError", "Avg. slope error: %f", avgSlopeError);
    LOG.write(LOG_LOW, "centerError", "Avg. center dist. error: %f", avgCenterDistError);
    LOG.write(LOG_LOW, "centerError", "Avg. intersect error: %f", avgIntersectError);

    LOG.write(LOG_LOW, "centerError", "Avg. slope certainty: %f", avgSlopeCertainty);
    LOG.write(LOG_LOW, "centerError", "Avg. center dist. certainty: %f", avgCenterDistCertainty);
    LOG.write(LOG_LOW, "centerError", "Avg. intersect certainty: %f", avgIntersectCertainty);

    if (slopeTurnCount > numGoodSlopeErrors / 2) {
        avgSlopeTurn = true;
    }

    if (centerDistTurnCount > numGoodCenterDistErrors / 2) {
        avgCenterDistTurn = true;
    }

    if (intersectTurnCount > numGoodIntersectErrors / 2) {
        avgIntersectTurn = true;
    }

    // based on our previous state, let's modify
    // the certainties based on empirical data
    // TODO: find actual certainty increments/decrements based
    //       off empirical data
    if (avgSlopeCertainty != 1.0 && numGoodSlopeErrors > 0) {
        if (avgSlopeTurn) {
            switch (prevTagState) {
            case TAGS_BOTH_GE_TWO:
                avgSlopeCertainty += 0.10;
                break;
            case TAGS_BOTH_ONE:
                avgSlopeCertainty += 0.10;
                break;
            case TAGS_ONE_OR_NONE:
                avgSlopeCertainty += 0.10;
                break;
            case TAGS_LESS_LEFT:
                avgSlopeCertainty += 0.10;
                break;
            case TAGS_LESS_RIGHT:
                avgSlopeCertainty += 0.10;
                break;
            }
        }
        else {
            switch (prevTagState) {
            case TAGS_BOTH_GE_TWO:
                avgSlopeCertainty += 0.10;
                break;
            case TAGS_BOTH_ONE:
                avgSlopeCertainty += 0.10;
                break;
            case TAGS_ONE_OR_NONE:
                avgSlopeCertainty += 0.10;
                break;
            case TAGS_LESS_LEFT:
                avgSlopeCertainty += 0.10;
                break;
            case TAGS_LESS_RIGHT:
                avgSlopeCertainty += 0.10;
                break;
            }
        }
    }

    if (avgCenterDistCertainty != 1.0 && numGoodCenterDistErrors > 0) {
        if (avgCenterDistTurn) {
            switch (prevTagState) {
            case TAGS_BOTH_GE_TWO:
                avgCenterDistCertainty += 0.10;
                break;
            case TAGS_BOTH_ONE:
                avgCenterDistCertainty += 0.10;
                break;
            case TAGS_ONE_OR_NONE:
                avgCenterDistCertainty += 0.10;
                break;
            case TAGS_LESS_LEFT:
                avgCenterDistCertainty += 0.10;
                break;
            case TAGS_LESS_RIGHT:
                avgCenterDistCertainty += 0.10;
                break;
            }
        }
        else {
            switch (prevTagState) {
            case TAGS_BOTH_GE_TWO:
                avgCenterDistCertainty += 0.10;
                break;
            case TAGS_BOTH_ONE:
                avgCenterDistCertainty += 0.10;
                break;
            case TAGS_ONE_OR_NONE:
                avgCenterDistCertainty += 0.10;
                break;
            case TAGS_LESS_LEFT:
                avgCenterDistCertainty += 0.10;
                break;
            case TAGS_LESS_RIGHT:
                avgCenterDistCertainty += 0.10;
                break;
            }
        }
    }

    if (avgIntersectCertainty != 1.0 && numGoodIntersectErrors > 0) {
        if (avgSlopeTurn) {
            switch (prevTagState) {
            case TAGS_BOTH_GE_TWO:
                avgIntersectCertainty += 0.10;
                break;
            case TAGS_BOTH_ONE:
                avgIntersectCertainty += 0.10;
                break;
            case TAGS_ONE_OR_NONE:
                avgIntersectCertainty += 0.10;
                break;
            case TAGS_LESS_LEFT:
                avgIntersectCertainty += 0.10;
                break;
            case TAGS_LESS_RIGHT:
                avgIntersectCertainty += 0.10;
                break;
            }
        }
        else {
            switch (prevTagState) {
            case TAGS_BOTH_GE_TWO:
                avgIntersectCertainty += 0.10;
                break;
            case TAGS_BOTH_ONE:
                avgIntersectCertainty += 0.10;
                break;
            case TAGS_ONE_OR_NONE:
                avgIntersectCertainty += 0.10;
                break;
            case TAGS_LESS_LEFT:
                avgIntersectCertainty += 0.10;
                break;
            case TAGS_LESS_RIGHT:
                avgIntersectCertainty += 0.10;
                break;
            }
        }
    }

    float turnCertainty = 0.0;
    float strafeCertainty = 0.0;
    if (avgSlopeTurn) {
        turnCertainty += avgSlopeCertainty;
    }
    else {
        strafeCertainty += avgSlopeCertainty;
    }

    if (avgCenterDistTurn) {
        turnCertainty += avgCenterDistCertainty;
    }
    else {
        strafeCertainty += avgCenterDistCertainty;
    }

    if (avgIntersectTurn) {
        turnCertainty += avgIntersectCertainty;
    }
    else {
        strafeCertainty += avgIntersectCertainty;
    }

    if (turnCertainty > strafeCertainty) {
        *turn = true;
    }
    else {
        *turn = false;
    }

    // TODO: if necessary, modify total error based on each certainty
    float totalError = 0.0;
    int numErrors = 0;
    if (*turn) {
        if (avgSlopeTurn) {
            numErrors++;
            totalError += avgSlopeError;
        }

        if (avgCenterDistTurn) {
            numErrors++;
            totalError += avgCenterDistError;
        }

        if (avgIntersectTurn) {
            numErrors++;
            totalError += avgIntersectError;
        }
    }
    else {
        if (!avgSlopeTurn) {
            numErrors++;
            totalError += avgSlopeError;
        }

        if (!avgCenterDistTurn) {
            numErrors++;
            totalError += avgCenterDistError;
        }

        if (!avgIntersectTurn) {
            numErrors++;
            totalError += avgIntersectError;
        }
    }

    if (numErrors == 0) {
        return 0.0;
    }

    return totalError / (float)numErrors;
}
/**************************************
 * Definition: Gives an error specifying how far away from the center
 *             of the squares (corridor) the rovio is, using both
 *             error of the slopes of seen squares and distance error
 *             of the two largest squares
 *
 * Parameters: The color of the squares we're supposed to be looking at
 *
 * Returns:    The center error in the interval [-1, 1], where 0 is no error
 *             A negative value is an indication to move right
 *             A positive value is an indication to move left
 **************************************/
float Camera::centerError(int color, bool *turn) {
    *turn = false;
    int slopeTurnCount = 0;
    int centerDistTurnCount = 0;
    int numGoodSlopeErrors = 0;
    int numGoodCenterDistErrors = 0;
    float totalGoodSlopeError = 0.0;
    float totalGoodCenterDistError = 0.0;

    // calculate slope and center distance errors the specified number
    // of times, ignoring -999's (which say they found nothing good)
    for (int i = 0; i < NUM_CAMERA_ERRORS; i++) {
        update();

        bool slopeTurn = false;
        bool centerDistTurn = false;

        float slopeError = corridorSlopeError(color, &slopeTurn);
        float centerDistError = centerDistanceError(color, &centerDistTurn);

        if (slopeError != -999) {
            numGoodSlopeErrors++;
            totalGoodSlopeError += slopeError;
            if (slopeTurn) {
                slopeTurnCount++;
            }
        }

        if (centerDistError != -999) {
            numGoodCenterDistErrors++;
            totalGoodCenterDistError += centerDistError;
            if (centerDistTurn) {
                centerDistTurnCount++;
            }
        }
    }

    float avgSlopeError = totalGoodSlopeError / (float)numGoodSlopeErrors;
    float avgCenterDistError = totalGoodCenterDistError / (float)numGoodCenterDistErrors;

    LOG.write(LOG_LOW, "centerError", "Avg. slope error: %f", avgSlopeError);
    LOG.write(LOG_LOW, "centerError", "Avg. center dist. error: %f", avgCenterDistError);
	
    // if we have good center distance errors, let's use those
	if (numGoodCenterDistErrors > 0) {
        // but are they still not optimal?
        if (avgCenterDistError > 0.25) {
            // center distance error is probably no longer a good indicator
            // of center error, so trust slope error now if we have it
            if (numGoodSlopeErrors > 0) {
                if (slopeTurnCount > numGoodSlopeErrors / 2) {
                    *turn = true;
                }
                return avgSlopeError;
            }
        }
        if (centerDistTurnCount > numGoodCenterDistErrors / 2) {
            *turn = true;
        }
        return avgCenterDistError;
    }

    // if we didn't have good center distance errors, let's
    // use slope error if we have it
    if (numGoodSlopeErrors > 0) {
        if (slopeTurnCount > numGoodSlopeErrors / 2) {
            *turn = true;
        }
        return avgSlopeError;
    }

    // otherwise, we didn't have good errors for either!
    *turn = true;
    return 1;
}

/**************************************
 * Definition: Gives an error specifying the difference of the distance 
 *             of the two largest squares from the center of the image
 *
 * Parameters: The color of the squares we're supposed to be looking at
 *
 * Returns:    An error in the interval [-1, 1], where 0 is no error,
 *             OR -999, which indicates a conclusion could not be reached
 *             A negative value is an indication to move right
 *             A positive value is an indication to move left
 **************************************/
float Camera::centerDistanceError(int color, bool *turn) {
    *turn = false;
    // find the center of the camera's image
    int width = thresholdedOf(color)->width;
    int center = width / 2;

    // find the largest squares on the left and right sides
    // of the image
    squares_t *leftSquare = biggestSquare(color, IMAGE_LEFT);
    squares_t *rightSquare = biggestSquare(color, IMAGE_RIGHT);
    
    // mark the squares so we can see them
    IplImage *bgr = getBGRImage();
    if (bgr != NULL) {
        markSquare(bgr, leftSquare, RED);
        markSquare(bgr, rightSquare, GREEN);
        // draw a line down the center of the image as well
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
            // we're probably just too far over on the
            // side of the larger square
            if (leftSquare->area > rightSquare->area) {
                // we should turn right slightly to 
                // unobstruct the right square
                *turn = true;
                return -0.25;
            }
            else {
                // we should turn left slightly
                *turn = true;
                return 0.25;

            }
        }
    }

    if (leftSquare == NULL && rightSquare == NULL) {
        // we couldn't find any squares
        return -999;
    } 
    else if (leftSquare == NULL) {
        // the left seems to be out of view, so we're
        // probably too far left. we should move right 
        return -0.5;
    } 
    else if (rightSquare == NULL) {
        // the right seems to be out of view, so we're
        // probably too far right. we should move left
        return 0.5;
    }

    // otherwise, we have two squares, so find the difference
    int leftError = center - leftSquare->center.x;
    int rightError = center - rightSquare->center.x;

    // return the difference in errors in range [-1, 1]
    *turn = true;
    return (float)(leftError + rightError) / (float)center;
}

/**************************************
 * Definition: Takes the perceived squares and performs a linear regression 
 *             on their locations of each side, and returns an error that is
 *             the difference in slopes of each side.
 *
 * Parameters: The color of the squares we're supposed to be looking at
 *
 * Returns:    An error in the interval [-1, 1], where 0 is no error,
 *             OR -999, which indicates a conclusion could not be reached
 *             A negative value is an indication to move right
 *             A positive value is an indication to move left
 **************************************/
float Camera::corridorSlopeError(int color, bool *turn) {
    *turn = false;
    // find a line of regression for each side of the image
    regressionLine leftSide = leastSquaresRegression(color, IMAGE_LEFT);
    regressionLine rightSide = leastSquaresRegression(color, IMAGE_RIGHT);
    regressionLine wholeImage = leastSquaresRegression(color, IMAGE_ALL);

    float xIntersect = 0;
    float yIntersect = 0;

    LOG.write(LOG_LOW, "slopeError", 
              "Left squares found: %d", leftSide.numSquares);
    LOG.write(LOG_LOW, "slopeError", 
              "Right squares found: %d", rightSide.numSquares);
    LOG.write(LOG_LOW, "slopeError", 
              "Left equation: y = %f*x + %f, r^2 = %f", leftSide.slope, leftSide.intercept, leftSide.rSquared);
    LOG.write(LOG_LOW, "slopeError", 
              "Right equation: y = %f*x + %f, r^2 = %f", rightSide.slope, rightSide.intercept, rightSide.rSquared);
    LOG.write(LOG_LOW, "slopeError", 
              "Total equation: y = %f*x + %f, r^2 = %f", wholeImage.slope, wholeImage.intercept, wholeImage.rSquared); 


    if(leftSide.numSquares >= 2 && rightSide.numSquares >= 2) {
       xIntersect = (rightSide.intercept - leftSide.intercept)/(leftSide.slope - rightSide.slope);
       yIntersect = leftSide.slope*xIntersect + leftSide.intercept;

       LOG.write(LOG_LOW, "slopeError",
      	         "Line intersection: (%f, %f)", xIntersect, yIntersect);
    } else {
       xIntersect = -999;
       yIntersect = -999;
    }
    
    // draw the lines of regression so we can see them
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

    // did we have enough squares on each side to find a line?
    if (leftSide.numSquares >= 2 && rightSide.numSquares >= 2) { 
        bool hasSlopeRight = false;
        bool hasSlopeLeft = false;

        // sanity-check the slopes to make sure we have good ones to go off of
		if (rightSide.slope > RIGHT_RIGHT_SLOPE && rightSide.slope < RIGHT_LEFT_SLOPE && 
            rightSide.slope != -0.500000 && rightSide.slope != 0.500000) {
			// seems like a good slope!
			hasSlopeRight = true;
		}

		if (leftSide.slope < LEFT_LEFT_SLOPE && leftSide.slope > LEFT_RIGHT_SLOPE && 
            leftSide.slope != -0.500000 && leftSide.slope != 0.500000) {
			// seems like a good slope!
			hasSlopeLeft = true;
		}
		
		if (hasSlopeLeft && hasSlopeRight) {
            float difference = leftSide.slope + rightSide.slope;

			if (difference > MAX_SLOPE_DIFFERENCE) {
                // the difference is large enough that we can say
                // the error is at its max, so we should move right
				return -1;
			} 
            else if (difference < -MAX_SLOPE_DIFFERENCE) {
                // we should move left
				return 1;
			} 
            else {
                // return the error in the range [-1, 1]
                *turn = true;
				return -difference / MAX_SLOPE;
			}
		}
/*
		if (hasSlopeLeft && !hasSlopeRight) {
			// no right slope, so interpolate based on left
            *turn = true;
            float leftTranslate = leftSide.slope - LEFT_MIDDLE_SLOPE;
			LOG.write(LOG_LOW, "slopeError", 
                      "only left slope, left translate: %f", leftTranslate);
			return leftTranslate;
		}
		
		if (!hasSlopeLeft && hasSlopeRight) {
            *turn = true;
			// no left slope, so interpolate based on right
			float rightTranslate= -(rightSide.slope - RIGHT_MIDDLE_SLOPE);
            LOG.write(LOG_LOW, "slopeError", 
                      "only right slope, right translate: %f", rightTranslate);
            return rightTranslate;
		}
*/
	}
    // we didn't have enough squares to be useful
    LOG.write(LOG_LOW, "slopeError", "no slopes!");
    return -999.0;
}

/**************************************
 * Definition: Performs a linear regression on the squares of 
 *             the specified side
 *
 * Algorithm Ref: http://mathworld.wolfram.com/LeastSquaresFitting.html
 *
 * Parameters: The color of the squares we're supposed to be looking at,
 *             and the side of the image to find squares on
 *
 * Returns:    A regressionLine struct representing the 
 *             calculated line of best fit
 **************************************/
regressionLine Camera::leastSquaresRegression(int color, int side) {
    regressionLine result;

    int width = thresholdedOf(color)->width;
    int center = width / 2;

    LOG.write(LOG_LOW, "regression", "image center: %d", center);

    result.numSquares = squareCount(color, side);
    
    // do we have enough squares to find a line?
    if (result.numSquares >= 2) {
        float xSum = 0.0;
        float ySum = 0.0;
        float xSqSum = 0.0;
        float xySum = 0.0;
	float ySqSum = 0.0;

        squares_t *curSquare = squaresOf(color);
        while (curSquare != NULL) {
	        switch (side) {
	        case IMAGE_LEFT:	
        	    if (curSquare->center.x < center) {
                            xSum += curSquare->center.x;
                            ySum += curSquare->center.y;
	    		    xSqSum += curSquare->center.x * curSquare->center.x;
	    		    xySum += curSquare->center.x * curSquare->center.y;
			    ySqSum += curSquare->center.y * curSquare->center.y;
		        }
		        break;
	        case IMAGE_RIGHT:
		        if (curSquare->center.x > center) {
	    		    xSum += curSquare->center.x;
	    		    ySum += curSquare->center.y;
	    		    xSqSum += curSquare->center.x * curSquare->center.x;
	    		    xySum += curSquare->center.x * curSquare->center.y;
			    ySqSum += curSquare->center.y * curSquare->center.y;
		        } 
		        break;
		case IMAGE_ALL:
	    		xSum += curSquare->center.x;
	    		ySum += curSquare->center.y;
	    		xSqSum += curSquare->center.x * curSquare->center.x;
	    		xySum += curSquare->center.x * curSquare->center.y;
			ySqSum += curSquare->center.y * curSquare->center.y;
			break;
	        }
	        curSquare = curSquare->next;
        }   

        float xAvg = xSum / result.numSquares;
        float yAvg = ySum / result.numSquares;

        result.intercept = ((yAvg * xSqSum) - (xAvg * xySum)) / 
                           (xSqSum - (result.numSquares * xAvg * xAvg));
        result.slope = (xySum - (result.numSquares * xAvg * yAvg)) / 
                       (xSqSum - (result.numSquares * xAvg * xAvg));
	result.rSquared = ((xySum - (result.numSquares * xAvg * yAvg))*(xySum - (result.numSquares * xAvg * yAvg)) / ((xSqSum - (result.numSquares * xAvg * xAvg)) * (ySqSum - (result.numSquares * yAvg * yAvg))));
    
    } else {
        // there aren't enough squares, so we error out the intercept and slope
        result.intercept = -999;
	result.slope = -999;
	result.rSquared = 0;
    }

    return result;
}

/**************************************
 * Definition: 	Takes a list of squares and returns it 
 * 		without any overlapping squares (largest square is kept)
 *
 * Parameters: 	squares_t containing a list of squares (from findSquares() call)
 * 		
 *
 * Returns: 	a squares_t containing the largest non-overlapping (distinct) squares
 * 		null if inputSquares is null
 * ************************************/
squares_t* Camera::rmOverlappingSquares(squares_t *inputSquares) {
    squares_t *sqHead = NULL;
    squares_t *newSquare = NULL;
    squares_t *sqLast = NULL;
    squares_t *iterator = NULL;
    squares_t *preIterator = NULL;

    LOG.write(LOG_HIGH, "rmOverlap", "entered rmOverlap method");

    while (inputSquares != NULL) { //Loop through all input squares once!
        if(sqHead == NULL) { //if clean list has not been started
	    sqHead = new squares_t;
            sqHead->area = inputSquares->area;
            sqHead->center.x = inputSquares->center.x;
            sqHead->center.y = inputSquares->center.y;
            sqHead->next = NULL;
        } else {
            //iterate through the clean list
            iterator = sqHead;
            while(iterator != NULL) {
                //check distance cutoff
                if(sqrt(pow(fabs(iterator->center.x - inputSquares->center.x),2.0)+pow(fabs(iterator->center.y - inputSquares->center.y),2.0)) < SQUARE_OVERLAP_DIST) {
                    if(inputSquares->area > iterator->area) {
                        newSquare = new squares_t;
                        newSquare->area = inputSquares->area;
                        newSquare->center.x = inputSquares->center.x;
                        newSquare->center.y = inputSquares->center.y;
                        if(iterator == sqHead) { //if we must replace the head
                            newSquare->next = sqHead->next;
                            sqHead = newSquare;
                        } else { //other insertion 
                            newSquare->next = iterator->next;
                            preIterator->next = newSquare;
                        }
                        break; //once square has been inserted, we don't need to keep comparing it
                    } 
                    //otherwise, don't store it
                } else {
                    //if not end of list (iterator while loop), continue
                    //if end of list, insert on end and break
                    if(iterator->next == NULL) {
                        newSquare = new squares_t;
                        newSquare->area = inputSquares->area;
                        newSquare->center.x = inputSquares->center.x;
                        newSquare->center.y = inputSquares->center.y;
                        newSquare->next = NULL;
                        iterator->next = newSquare;
                        
                        //just want to make it fall off the end of the list
                        iterator = iterator->next;
                    }
                }
                preIterator = iterator;
                iterator = iterator->next;
            }
        }
        inputSquares = inputSquares->next; 
    } 
    return sqHead; //Returns a list of distinct squares 
}

/**************************************
 * Definition: Checks if two squares are on the same plane
 *
 * Parameters: A left and right square
 *
 * Returns:    true or false
 **************************************/
bool Camera::onSamePlane(squares_t *leftSquare, squares_t *rightSquare) {
    float slope = (float)(leftSquare->center.y - rightSquare->center.y) / 
                  (float)(leftSquare->center.x - rightSquare->center.x);
    return (fabs(slope) <= MAX_PLANE_SLOPE);
}

/**************************************
 * Definition: Finds the biggest square of the specified color
 *             on the specified side of the image
 *
 * Parameters: the color to threshold by and the side of the image
 *
 * Returns:    the biggest square
 **************************************/
squares_t* Camera::biggestSquare(int color, int side) {
    squares_t *largestSquare = NULL;

    int width = thresholdedOf(color)->width;
    int center = width / 2;

    squares_t *curSquare = squaresOf(color);
    while (curSquare != NULL) {
        if ((side == IMAGE_LEFT && curSquare->center.x < center) ||
            (side == IMAGE_RIGHT && curSquare->center.x > center)) {
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

/**************************************
 * Definition: Counts the number of squares of the specified color
 *             on the specified side of the image there are
 *
 * Parameters: the color to threshold by and the side of the image
 *
 * Returns:    an int with the count
 **************************************/
int Camera::squareCount(int color, int side) {
    int squareCount = 0;

    int width = thresholdedOf(color)->width;
    int center = width / 2;

    // iterate through the squares and count how many there are
    // on the specified side of the image
    squares_t *curSquare = squaresOf(color);
    while (curSquare != NULL) {
        switch (side) {
        case IMAGE_LEFT: 
            if (curSquare->center.x < center) { 
                LOG.write(LOG_LOW, "squareCount",
                          "Left square - x: %d y: %d area: %d",
                          curSquare->center.x, curSquare->center.y, curSquare->area);
                squareCount++;
            }
            break;
        case IMAGE_RIGHT:
            if (curSquare->center.x > center) {
                LOG.write(LOG_HIGH, "squareCount",
                          "Right square - x: %d y: %d area: %d", 
                          curSquare->center.x, curSquare->center.y, curSquare->area);
                squareCount++;
            }
            break;
        }
        curSquare = curSquare->next;
    }

    return squareCount;
}

/**************************************
 * Definition: Returns the stored thresholded image of the given color
 *
 * Parameters: the color to threshold by
 *
 * Returns:    the thresholded IplImage
 **************************************/
IplImage* Camera::thresholdedOf(int color) {
    IplImage *thresholded = NULL;
    switch (color) {
    case COLOR_PINK:
        thresholded = _pinkThresholded;
        break;
    case COLOR_YELLOW:
        thresholded = _yellowThresholded;
        break;
    }
    return thresholded;
}

/**************************************
 * Definition: Returns the stored squares of the given color
 *
 * Parameters: the color to threshold by
 *
 * Returns:    a squares_t linked list
 **************************************/
squares_t* Camera::squaresOf(int color) {
    squares_t *squares = NULL;
    switch (color) {
    case COLOR_PINK:
        squares = _pinkSquares;
        break;
    case COLOR_YELLOW:
        squares = _yellowSquares;
        break;
    }
    return squares;
}

/**************************************
 * Definition: Finds squares of the given color and given minimum size
 *
 * Parameters: the color to threshold by and the minimum area for a square
 *
 * Returns:    a squares_t linked list
 **************************************/
squares_t* Camera::findSquaresOf(int color, int areaThreshold) {
    squares_t *squares = NULL;
    switch (color) {
    case COLOR_PINK:
        squares = findSquares(_pinkThresholded, areaThreshold);
        break;
    case COLOR_YELLOW:
        squares = findSquares(_yellowThresholded, areaThreshold);
        break;
    }
    return rmOverlappingSquares(squares);
}

/**************************************
 * Definition: Finds squares in an image with the given minimum size
 *
 * (Taken from the API and modified slightly)
 * Doesn't require exactly 4 sides, convexity or near 90 deg angles either ('findBlobs')
 *
 * Parameters: the image to find squares in and the minimum area for a square
 *
 * Returns:    a squares_t linked list
 **************************************/
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
    //cvPyrDown( pyr, pyr2, CV_GAUSSIAN_5x5 );
    //cvPyrUp( pyr2, pyr, CV_GAUSSIAN_5x5 );
    cvPyrUp( pyr, img, CV_GAUSSIAN_5x5 );

    // Apply the canny edge detector and set the lower to 0 (which forces edges merging) 
    cvCanny(img, canny, 0, 50, 3);
        
    // Dilate canny output to remove potential holes between edge segments 
    cvDilate(canny, canny, 0, 2);
        
    // Find the contours and store them all as a list
    // was CV_RETR_EXTERNAL
    cvFindContours(canny, storage, &contours, sizeof(CvContour), 
                   CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));
            
    // Test each contour to find squares
    while (contours) {
        // Approximate a contour with accuracy proportional to the contour perimeter
        result = cvApproxPoly(contours, sizeof(CvContour), storage, 
                              CV_POLY_APPROX_DP, cvContourPerimeter(contours)*0.1, 0 );
        // Square contours should have
        //  * 4 vertices after approximation
        //  * Relatively large area (to filter out noisy contours)
        //  * Ne convex.
        // Note: absolute value of an area is used because
        // area may be positive or negative - in accordance with the
        // contour orientation
        if (result->total >= 4 && 
            fabs(cvContourArea(result,CV_WHOLE_SEQ,0)) > areaThreshold /*&& 
            cvCheckContourConvexity(result)*/) {
            s=0;
            for(i=0; i<5; i++) {
                            // Find the minimum angle between joint edges (maximum of cosine)
                if(i >= 2) {
                    t = fabs(ri_angle((CvPoint*)cvGetSeqElem(result, i),
                                      (CvPoint*)cvGetSeqElem(result, i-2),
                                      (CvPoint*)cvGetSeqElem( result, i-1 )));
                    s = s > t ? s : t;
                }
            }

            // If cosines of all angles are small (all angles are ~90 degree) then write the vertices to the sequence 
            //if( s < 0.2 ) {
                for( i = 0; i < 4; i++ ) {
                    cvSeqPush(squares, (CvPoint*)cvGetSeqElem(result, i));
                }
            //}
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

/**************************************
 * Definition: Grabs a new HSV image from the camera
 *
 * Returns:    an IplImage in HSV format
 **************************************/
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

/**************************************
 * Definition: Grabs a new thresholded image from the camera
 *
 * Parameters: low and high scalars specifying the threshold color range
 *
 * Returns:    a thresholded IplImage
 **************************************/
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

/**************************************
 * Definition: Grabs a new BGR image from the camera
 *
 * Returns:    an IplImage in BGR format
 **************************************/
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
