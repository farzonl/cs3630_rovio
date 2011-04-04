#include "Configuration.h"
#ifdef ROVIO
#ifndef ROBOT_INTERFACE_R
#define ROBOT_INTERFACE_R
#include "curl/curl.h"

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

typedef enum horizontal_class {
    _left = 0,
    center,
    _right
} horizontal_class;

typedef enum vertical_class {
    low = 0,
    middle,
    high
} vertical_class;

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

int rovio_drive(CURL *curl, int n, RovioDirection direction);
int rovio_turn(CURL *curl, horizontal_class direction, int n);

void robotWait();


#endif
#endif
