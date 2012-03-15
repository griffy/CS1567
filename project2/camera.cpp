#include "camera.h"
#include "logger.h"
#include <robot_color.h>

Camera::Camera(RobotInterface *robotInterface) {
    _robotInterface = robotInterface;
    setQuality(RI_CAMERA_QUALITY_HIGH);
    setResolution(RI_CAMERA_RES_320);
    _pinkThresholded = NULL;
    _yellowThresholded = NULL;
    _pinkSquares = NULL;
    _yellowSquares = NULL;
}

Camera::~Camera() {
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
    _pinkThresholded = getThresholdedImage(RC_PINK_LOW, RC_PINK_HIGH);
    _yellowThresholded = getThresholdedImage(RC_YELLOW_LOW, RC_YELLOW_HIGH);
    _pinkSquares = findSquaresOf(COLOR_PINK, DEFAULT_SQUARE_SIZE);
    _yellowSquares = findSquaresOf(COLOR_YELLOW, DEFAULT_SQUARE_SIZE);
}

/* Error is defined to be the distance of the square
   from the center of the camera
 */
int Camera::leftSquareDistanceError(int color) {
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
    squares_t *cur_square = squares;
    squares_t *largest_square;
    while (cur_square != NULL) {
        if (cur_square->center.x < center) {
            if (largest_square == NULL) {
                largest_square = cur_square;
            }
            else {
                if (cur_square->area > largest_square->area) {
                    largest_square = cur_square;
                }
            }
        }
        cur_square = cur_square->next;
    }

    if (largest_square == NULL) {
        return -1;
    }

    return center - largest_square->center.x;
}

int Camera::rightSquareDistanceError(int color) {
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
    squares_t *cur_square = squares;
    squares_t *largest_square;
    while (cur_square != NULL) {
        if (cur_square->center.x > center) {
            if (largest_square == NULL) {
                largest_square = cur_square;
            }
            else {
                if (cur_square->area > largest_square->area) {
                    largest_square = cur_square;
                }
            }
        }
        cur_square = cur_square->next;
    }

    if (largest_square == NULL) {
        return -1;
    }

    return largest_square->center.x - center;
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

#ifdef DEBUG_SQUARE_CANNY
    cvShowImage("Debug - CANNY", canny);
#endif
        
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
