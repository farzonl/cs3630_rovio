#ifndef ROBOT_INTERFACE
#define ROBOT_INTERFACE

#include "Configuration.h"


#ifdef PLEO
#include "RobotInterfacePleo.h"
#endif

#ifdef ROVIO
#include "RobotInterfaceRovio.h"
#endif


#endif