#ifndef CS1567_CONSTANTS_H
#define CS1567_CONSTANTS_H

#define PI 3.14159265358979323846

#define DEGREE_20 0.34906585
#define DEGREE_25 0.436332313
#define DEGREE_30 0.523598776 // pi/6
#define DEGREE_60 1.04719755 // pi/6
#define DEGREE_90 1.57079633 // pi/2
#define DEGREE_150 2.617993878 // 5pi/6

#define ROBOT_DIAMETER 29 // cm

#define MAX_UPDATE_FAILS 5 // max allowable fails to update robot interface

// Kalman uncertainties
#define PROC_X_UNCERTAIN 0.05
#define PROC_Y_UNCERTAIN 0.05
#define PROC_THETA_UNCERTAIN 0.1

#define NS_X_UNCERTAIN 0.025
#define NS_Y_UNCERTAIN 0.025
#define NS_THETA_UNCERTAIN 0.1

#define WE_X_UNCERTAIN 0.05
#define WE_Y_UNCERTAIN 0.01
#define WE_THETA_UNCERTAIN 0.025 // was 0.05

// PID gains min and max
#define MIN_DIST_GAIN -0.1
#define MAX_DIST_GAIN 0.1
#define MIN_THETA_GAIN -0.05
#define MAX_THETA_GAIN 0.05
#define MIN_CENTER_GAIN -0.05
#define MAX_CENTER_GAIN 0.05
#define MIN_SLOPE_GAIN -0.05
#define MAX_SLOPE_GAIN 0.05

// PID tuning constants
#define PID_DIST_KP 0.8
#define PID_DIST_KI 0.05
#define PID_DIST_KD 0.05
#define PID_THETA_KP 0.65
#define PID_THETA_KI 0.001
#define PID_THETA_KD 0.001
#define PID_CENTER_KP 0.03
#define PID_CENTER_KI 0.0001
#define PID_CENTER_KD 0.0001
#define PID_SLOPE_KP 0.65
#define PID_SLOPE_KI 0.001
#define PID_SLOPE_KD 0.001

// acceptable proximities from base
#define MAX_DIST_ERROR 35.0 // in cm
#define MAX_THETA_ERROR DEGREE_30

// acceptable threshold for being in the center of squares, in pixels
#define MAX_CENTER_ERROR 10
// how close within 0 we can be before adjusting
#define MAX_SLOPE_ERROR 0.05 

// the largest filter size (used for prefilling data)
#define MAX_FILTER_TAPS 7

#define ROOM_2 0
#define ROOM_3 1
#define ROOM_4 2
#define ROOM_5 3

#define WE_SCALE 4.0 // (avg) ticks per cm

// the average ticks per cm in x and y per room
const float NS_ROOM_SCALE[4][2] =  {{49.2, 37.1}, 
                 				    {45.4, 57.6}, 
				                    {59.6, 36.7}, 
			                	    {37.4, 53.5}};  //Rosie Data
			   

// ROTATION is angle relative to room 2's base where 0 degrees is parallel to far wall
// Theta increases counter-clockwise     |
//    |-door-|                           |
//                     __4__             |
//                    |     |		     |
//                  3 |     | 5          |
//                    |*____|            |
//                       2               |
//     theta = pi/2                      |
//     ^                                 |
//     |                                 |
//     y                                 |
//     |                                 |
//     *-x--> theta = 0                  |
//     ^                                 |
//     origin                            |
//_______________________________________|

const float NS_ROOM_ROTATION[4] = {0, 1.5708, 0.0, 1.6005}; //Rosie data, radians, correct!

// the distance of column top-right corner from base 0
const float COL_OFFSET[2] = {193.0, 234.0};
// the distances of ns origins from column corner (labeled with a * in above map)
const float NS_ROOM_ORIGINS_FROM_COL[4][2] = {{18.0, -124.0},
										      {-147.0, 8.0},
										      {41.0, 168.0},
										      {196.0, 30.0}};


const float THETA_SHIFT[4] = {-1.5708, 1.5708, -1.5708, 1.5708}; //Value necessary to push the theta = 0 onto the x-axis rather than the y


#endif
