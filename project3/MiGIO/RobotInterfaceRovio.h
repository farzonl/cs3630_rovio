#include "Configuration.h"
#ifdef ROVIO
#ifndef ROBOT_INTERFACE_R
#define ROBOT_INTERFACE_R

static char robotImage[] = "Data/RovioHeader.jpg";

static int numActuators = 2;
static int actuatorValues[] = {0,0};
static const char* actuatorNames[] = {" 0)Cmd"," 1)Speed"};

static int numBinarySensors = 0;
static bool binarySensors[] = {0};
static char* binaryNames[] = {0};

static int numContSensors = 3;
static int contSensors[] = {0,0,0};
static const char* contNames[] = {" 0)X"," 1)Y"," 2)THETA"};

void robotReadSensors();
void robotController();
void robotSendActuators();
void robotWait();

typedef enum horizontal_class {
    TurnLeft = 0,
    TurnRight = 2
} horizontal_class;

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
void rovio_turn(horizontal_class direction, int n);
void rovio_camera_height(vertical_class height);

#endif
#endif
