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
void turnLeft(int angle);
void turnRight(int angle);
void forward();
void backward();
void slideLeft();
void slideRight();
void slideUpRight();
void slideUpLeft();
void slideDownRight();
void slideDownLeft();
void robotController();
void robotSendActuators();
void robotWait();


#endif
#endif
