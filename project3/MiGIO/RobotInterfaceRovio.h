#include "Configuration.h"
#ifdef ROVIO
#ifndef ROBOT_INTERFACE_R
#define ROBOT_INTERFACE_R

void robotReadSensors();
void robotController();
void robotSendActuators();
void robotWait();

typedef enum RovioTurn {
    TurnLeft = 0,
    TurnRight = 2
} RovioTurn;

typedef enum RovioDirection {
    DirNone = 0,
    DirForward = 1,
    DirBackward,
    DirLeft,
    DirRight,
    DirLeftForward = 7,
    DirRightForward,
    DirLeftBackward,
    DirRightBackward
} RovioDirection;

typedef enum vertical_class {
    low = 0,
    middle,
    high
} vertical_class;

void rovio_drive(int n, RovioDirection direction);
// direction = Left or Right
// n = 3 for 45, 7 for 90, technically in increments of 20 degrees
void rovio_turn(RovioTurn direction, int n);
void rovio_turn_small(RovioTurn direction);
void rovio_camera_height(vertical_class height);

#endif
#endif
