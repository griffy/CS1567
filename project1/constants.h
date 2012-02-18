#ifndef CS1567_CONSTANTS_H
#define CS1567_CONSTANTS_H

#define PI 3.14159265358979323846
#define DEGREE_30 0.523598776 // pi/6
#define DEGREE_60 1.04719755 // pi/6
#define DEGREE_90 1.57079633 // pi/2
#define DEGREE_150 2.617993878 // 5pi/6

#define WE_TICKS 4.0 // (avg) ticks per cm
#define NS_TICKS 45.0 // (avg) ticks per cm, not really used

#define ROBOT_DIAMETER 29 // cm

#define MAX_NUM_FAILS 5

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

#define MAX_THETA_ERROR 0.5236 // 30 degrees
//#define MAX_THETA_ERROR 0.26 // 15 degrees
#define MAX_DIST_ERROR 15.0 // in cm

// TODO: find out proper indexes for arrays below
#define ROOM_2 0
#define ROOM_3 1
#define ROOM_4 2
#define ROOM_5 3

// ROOM 2 stuff
//const float ROOM_X_SHIFT[4] = {223, 48, 234, 389};
//const float ROOM_Y_SHIFT[4] = {89, 242, 402, 264};
//const float ROOM_SCALE[2][4] = {{27.2, 45.4, 59.6, 37.4}, 
//								  {20.3, 57.6, 36.7, 53.5}}; //Rosie Data

const float ROOM_SCALE[4][2] = {{27.2, 20.3},
								{45.4, 57.6},
							    {59.6, 36.7},
							    {37.4, 53.5}};
							   
// store the distance of column top-right corner from base 0
const float COL_OFFSET[2] = {193.0, 234.0};
// store the distances of origins from column top-right corner
const float ROOM_ORIGINS_FROM_COL[4][2] = {{30.0, -145.0}, 
										   {-147.0, 8.0},
										   {41.0, 168.0},
										   {196.0, 30.0}};

//const float ROOM_X_SHIFT[4]= {310, 48, 229, 375};
//const float ROOM_Y_SHIFT[4]= {255, 281, 449, 303};
//const float ROOM_SCALE[2][4]= {{27.2, 45.4, 59.6, 37.4}, // x
//							   {27.3, 57.6, 36.7, 53.5}}; // y (based on Rosie Data)
const float ROOM_ROTATION[4] = {77.3, 0, 92.8, 3.4}; //Rosie Data
			      //{80.7, -6.9, 95.7, 3.4}; Optimus Data
const float ROOM_FLIPX[4] = {true, true, false, false}; //binary flag indicating whether to reflect x-coordinates over y-axis
const float ROOM_FLIPY[4] = {false, false, true, true}; //binary flag indicating whether to reflect y-coordinates over x-axis


// ROTATION is angle relative to room 2's base where 0 degrees is parallel to far wall
//                                       |
//                     _4_               |
//                    |   |              |
//                  3 |___| 5            |
//                      2                |
//     theta = 90                        |
//     ^                                 |
//     |                                 |
//     y                                 |
//     |                                 |
//     *-x--> theta = 0                  |
//________________________________________
// ROOM_ROTATION[0]=350.7;
// ROOM_ROTATION[1]=263.1;
// ROOM_ROTATION[2]=5.7;
// ROOM_ROTATION[3]=273.4;

#endif
