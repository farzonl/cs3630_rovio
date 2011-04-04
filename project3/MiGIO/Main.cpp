//---------------------------------------------------------------------
//  Copyright (c) 2010 Mike Stilman
//  All Rights Reserved.
//
//  Permission to duplicate or use this software in whole or in part
//  is only granted by consultation with the author.
//
//    Mike Stilman              mstilman@cc.gatech.edu
//
//	  Robotics and Intelligent Machines
//    Georgia Tech
//--------------------------------------------------------------------

#include <cv.h>
#include "curl/curl.h"
#include <cxcore.h>
#include <highgui.h>
#include <iostream>
#include <stdio.h>
#ifdef _WIN32
#include <process.h>
#else
#include <pthread.h>
#endif
#include <fstream>

using namespace std;

#include "CVConsole.h"
#include "Configuration.h"
#include "RobotStatus.h"
#include "HTTPInterface.h"
#include "CameraInterface.h"
#include "RobotInterface.h"

int main(void)
{
  http_interface_init();
  CURL *curl;
  curl = curl_easy_init();
  

  /*initCameras();*/
#ifdef _WIN32
  //_beginthread(cameraThread,0,(void*)0);
#else
  pthread_t thread;
  pthread_create(&thread, NULL, &cameraThread, NULL);
#endif

  initConsole();
  initStatusWindow();

 #ifdef USE_CONTROLLER
	  robotReadSensors();  // Blank functions
	  robotController();
#endif

  while(true){
	  key = cvWaitKey(30); // Really a sleep with input
	  if(key == 'q') break;

	  /*
#ifdef USE_CAMERA
	  processCamera(); // Blank function
#endif
	  */

#ifdef USE_CONTROLLER
	//robotSendActuators();
	  if(curl) {
		   //rovio_forward(curl, 18,5);
		  //rovio_driveLeft(curl,10); 
		  //rovio_driveRight(curl,10);
		  //rovio_turnRightByDegree(curl,4);
		  //rovio_turnLeftByDegree(curl,4);
	  }

#endif

	  //updateConsole();
	  updateStatusWindow();
  }

  //robotWait();
  //destroyConsole();
  destroyStatusWindow();
  http_interface_destroy();
  curl_easy_cleanup(curl);

  return 0;
}
