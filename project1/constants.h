#ifndef CS1567_CONSTANTS_H
#define CS1567_CONSTANTS_H

#define PI 3.14159265358979323846
#define DEGREE_30 0.523598776 // pi/6
#define DEGREE_60 1.04719755 // pi/6
#define DEGREE_150 2.617993878 // 5pi/6

#define WE_TICKS 4 // ticks per cm
#define NS_TICKS 45 // ticks per cm

#define ROBOT_DIAMETER 29 // cm

// ROOM 2 stuff
const float ROOM_X_SHIFT[4]= {199, 48, 229, 375};
const float ROOM_Y_SHIFT[4]= {154, 281, 449, 303};
const float ROOM_SCALE[2][4]= {{27.2, 45.4, 59.6, 37.4}, {27.3, 57.6, 36.7, 53.5}}; //Rosie Data
const float ROOM_ROTATION[4]= 	{77.3, 0, 92.8, 3.4}; //Rosie Data
			      //{80.7, -6.9, 95.7, 3.4}; Optimus Data
const float ROOM_FLIPX[4] = {1,1,0,0}; //binary flag indicating whether to reflect x-coordinates over y-axis
const float ROOM_FLIPY[4] = {0,0,1,1}; //binary flag indicating whether to reflect y-coordinates over x-axis
// 
// //ROOM_SCALE[0] is room 2 scale => # ticks to cm in room 2
// //X scale
// ROOM_SCALE[0][0]=45;
// ROOM_SCALE[0][1]=45;
// ROOM_SCALE[0][2]=45;
// ROOM_SCALE[0][3]=45;
// //Y scale
// ROOM_SCALE[1][0]=45;
// ROOM_SCALE[1][1]=45;
// ROOM_SCALE[1][2]=45;
// ROOM_SCALE[1][3]=45;


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
