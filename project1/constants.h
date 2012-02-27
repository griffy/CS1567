#ifndef CS1567_CONSTANTS_H
#define CS1567_CONSTANTS_H

#define PI 3.14159265358979323846

#define DEGREE_20 0.34906585
#define DEGREE_25 0.436332313
#define DEGREE_30 0.523598776 // pi/6
#define DEGREE_60 1.04719755 // pi/6
#define DEGREE_90 1.57079633 // pi/2
#define DEGREE_150 2.617993878 // 5pi/6

#define WE_TICKS 4.0 // (avg) ticks per cm
#define NS_TICKS 45.0 // (avg) ticks per cm, not really used

#define ROBOT_DIAMETER 29 // cm

#define MAX_UPDATE_FAILS 5

#define MIN_DIST_GAIN -0.1
#define MAX_DIST_GAIN 0.1

#define MIN_THETA_GAIN -0.05
#define MAX_THETA_GAIN 0.05

#define PID_DIST_KP 0.8
#define PID_DIST_KI 0.05
#define PID_DIST_KD 0.05

#define PID_THETA_KP 0.65
#define PID_THETA_KI 0.001
#define PID_THETA_KD 0.001

#define MAX_FILTER_TAPS 7

#define MAX_DIST_ERROR 35.0 // in cm
#define MAX_THETA_ERROR DEGREE_30

#define ROOM_2 0
#define ROOM_3 1
#define ROOM_4 2
#define ROOM_5 3

// the average ticks per cm in x and y per room
const float ROOM_SCALE[4][2] =  {{49.2, 37.1}, 
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

const float ROOM_ROTATION[4] = {0, 1.5708, 0.0, 1.6005}; //Rosie data, radians, correct!

// the distance of column top-right corner from base 0
const float COL_OFFSET[2] = {193.0, 234.0};
// the distances of origins from column corner (labeled with a * in above map)
const float ROOM_ORIGINS_FROM_COL[4][2] = {{18.0, -124.0},
										   {-147.0, 8.0},
										   {41.0, 168.0},
										   {196.0, 30.0}};


const float THETA_SHIFT[4] = {-1.5708, 1.5708, -1.5708, 1.5708}; //Value necessary to push the theta = 0 onto the x-axis rather than the y


#endif
