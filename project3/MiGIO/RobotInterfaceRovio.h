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


CURLcode rovio_turnRightByDegree(CURL *curl, int n);
CURLcode rovio_forward(CURL *curl, int n,int s);
CURLcode rovio_driveLeft(CURL *curl, int n);
CURLcode rovio_driveRight(CURL *curl, int n);
CURLcode rovio_turnLeftByDegree(CURL *curl, int n);

void robotWait();


#endif
#endif
