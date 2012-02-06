#define PI 3.1415926

// ROOM 2 stuff
float ROOM_X_SHIFT[4];
float ROOM_Y_SHIFT[4];
float ROOM_SCALE[4];
float ROOM_ROTATION[4];

ROOM_X_SHIFT[0]=199;
ROOM_Y_SHIFT[0]=154;

ROOM_X_SHIFT[1]=48;
ROOM_Y_SHIFT[1]=281;

ROOM_X_SHIFT[2]=229;
ROOM_Y_SHIFT[2]=449;

ROOM_X_SHIFT[3]=362;
ROOM_Y_SHIFT[3]=341;

//ROOM_SCALE[0] is room 2 scale => # ticks to cm in room 2
ROOM_SCALE[0]=45;
ROOM_SCALE[1]=45;
ROOM_SCALE[2]=45;
ROOM_SCALE[3]=45;

// ROTATION is angle relative to room 2's base where 0 degrees is parallel to far wall
ROOM_ROTATION[0]=45;
ROOM_ROTATION[1]=45;
ROOM_ROTATION[2]=45;
ROOM_ROTATION[3]=45;

#define NUMBASES 5
Pose * bases[NUMBASES];
Pose[0] = new Pose(0,0,0);
Pose[1] = new Pose(0,0,0);
Pose[2] = new Pose(0,0,0);
Pose[3] = new Pose(0,0,0);
Pose[4] = new Pose(0,0,0);
