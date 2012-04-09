/**
 * constants.h
 * 
 * @brief 
 * 		This header defines constants for use throughout the project
 * 
 * @author
 * 		Tom Nason
 * 		Joel Griffith
 * 		Shawn Hanna
 * 
 **/

#ifndef CS1567_CONSTANTS_H
#define CS1567_CONSTANTS_H

#define PI 3.14159265358979323846

#define DEGREE_0 0.0
#define DEGREE_20 0.34906585
#define DEGREE_25 0.436332313
#define DEGREE_30 0.523598776 // pi/6
#define DEGREE_45 0.785398163 // pi/4
#define DEGREE_60 1.04719755 // pi/3
#define DEGREE_90 1.57079633 // pi/2
#define DEGREE_150 2.617993878 // 5pi/6
#define DEGREE_180 3.14159265 // pi
#define DEGREE_270 4.71238898 // 3pi/2

#define ROBOT_DIAMETER 29 // cm

#define MAX_UPDATE_FAILS 5 // max allowable fails to update robot interface

// Kalman uncertainties
// process uncertainties
#define PROC_X_UNCERTAIN 0.25
#define PROC_Y_UNCERTAIN 0.25
#define PROC_THETA_UNCERTAIN 0.25

// north star uncertainties
#define NS_X_UNCERTAIN 0.15
#define NS_Y_UNCERTAIN 0.25
#define NS_THETA_UNCERTAIN 0.05

// wheel encoder uncertainties
#define WE_X_UNCERTAIN 0.05
#define WE_Y_UNCERTAIN 0.05
#define WE_THETA_UNCERTAIN 0.15

// PID gains min and max
#define MIN_DIST_GAIN -0.1
#define MAX_DIST_GAIN 0.1
#define MIN_THETA_GAIN -0.05
#define MAX_THETA_GAIN 0.05
#define MIN_CENTER_GAIN -0.05
#define MAX_CENTER_GAIN 0.05
#define MIN_TURN_CENTER_GAIN -0.05
#define MAX_TURN_CENTER_GAIN 0.05

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
#define PID_TURN_CENTER_KP 0.65
#define PID_TURN_CENTER_KI 0.001
#define PID_TURN_CENTER_KD 0.001

// acceptable proximities from base
#define MAX_DIST_ERROR 10.0 // in cm
#define MAX_THETA_ERROR DEGREE_20

// acceptable threshold for being in the center of squares (out of 1)
#define MAX_CENTER_ERROR 0.1
#define MAX_TURN_CENTER_ERROR 0.1

// the largest filter size (used for prefilling data)
#define MAX_FILTER_TAPS 7

#define ROOM_2 0
#define ROOM_3 1
#define ROOM_4 2
#define ROOM_5 3

#define WE_SCALE 4.0 // (avg) ticks per cm

/* north star transformation constants */

// Notes regarding room scale values:
//   Rooms 2 and 3 Y values vary greatly depending on robot orientation within 
//   the corridor (moving via strafe or straight)
//	 
//   Larger when straight, smaller when strafe
//	 Room 5 y varies similarly
//	 Room 2 may be dodgy at end of corridor (X and Y)
//	 High quality values: NS2x, NS5x, (lesser so) NS3x, NS5y

// the average ticks per cm in x and y per room per robot
const float NS_ROOM_SCALE[6][4][2] = {
	// Rosie
	{{49.2, 37.1}, 
     {45.4, 57.6}, 
	 {59.6, 36.7}, 
	 {37.4, 53.5}},
	// Bender
	{{58.1, 52.3},		
	 {58.8, 47.5},
	 {59.6, 36.7},
	 {53.6, 71.7}},
	// Johnny5 (using Bender's)
	{{58.1, 52.3},		
	 {58.8, 47.5},
	 {59.6, 36.7},
	 {53.6, 71.7}},
	// Optimus (using Bender's)
	{{58.1, 52.3},		
	 {58.8, 47.5},
	 {59.6, 36.7},
	 {53.6, 71.7}},
	// WallE (using Bender's)
	{{58.1, 52.3},		
	 {58.8, 47.5},
	 {59.6, 36.7},
	 {53.6, 71.7}},
	// Gort (using Bender's)
	{{58.1, 52.3},		
	 {58.8, 47.5},
	 {59.6, 36.7},
	 {53.6, 71.7}}
};

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

const float NS_ROOM_ROTATION[6][4] = {
	// Rosie
	{0.0, 
     1.5708, 
     0.0, 
     1.6005},
	// Bender
	{0.0, 
     1.6708,
     0.15, 
     1.6005},
	// Johnny5 (using Bender's)
	{0.0, 
     1.6708, 
     0.15, 
     1.6005},
	// Optimus (using Bender's)
	{0.0, 
     1.6708, 
     0.15, 
     1.6005},
	// WallE (using Bender's)
	{0.0, 
     1.6708, 
     0.15, 
     1.6005},
    // Gort (using Bender's)
    {0.0,
     1.6708,
     0.15,
     1.6005}
};

// the distance of column top-right corner from base 0 in cm
const float COL_OFFSET[2] = {193.0, 234.0};

// the distances of ns origins from column corner (labeled with a * in above map)
// Note: Don't trust room 4 for Gort or Bender
const float NS_ROOM_ORIGINS_FROM_COL[6][4][2] = {
	// Rosie
	{{18.0, -124.0},
	 {-147.0, 8.0},
	 {41.0, 168.0},
	 {196.0, 30.0}},
	// Bender
	{{36.0,-125.0}, 
	 {-116.0,18.0}, 
	 {62.0, 174.0}, 
	 {193.0, 36.0}},
	// Johnny5
	{{5.0, -104.0}, 
	 {-144.0, 25.0}, 
	 {32.0, 200.0}, 
	 {195.0, 32.0}},
	// Optimus (using Bender's)
	{{36.0,-125.0}, 
	 {-116.0,18.0}, 
	 {62.0, 174.0}, 
	 {193.0, 36.0}},
	// WallE (using Bender's)
	{{36.0,-125.0}, 
	 {-116.0,18.0}, 
	 {62.0, 174.0}, 
	 {193.0, 36.0}},
	// Gort
	{{48.0,-97.0}, 
	 {-95.0,32.0}, 
	 {84.0, 191.0}, 
	 {212.0, 102.0}}
};


// Value necessary to push the theta = 0 onto the x-axis rather than the y					  
const float THETA_SHIFT[6][4] = {
	// Rosie
	{1.5708, 
	 -1.5708, 
	 1.5708, 
	 -1.5708},
	// Bender (using Rosie's)
	{1.5708, 
	 -1.5708, 
	 1.5708, 
	 -1.5708},
	// Johnny5 (using Rosie's)
	{1.5708, 
	 -1.5708, 
	 1.5708, 
	 -1.5708},
	// Optimus (using Rosie's)
	{1.5708, 
	 -1.5708, 
	 1.5708, 
	 -1.5708},
	// WallE (using Rosie's)
	{1.5708, 
	 -1.5708, 
	 1.5708, 
	 -1.5708},
	// Gort (using Rosie's)
	{1.5708, 
	 -1.5708, 
	 1.5708, 
	 -1.5708}
};

#endif
