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
#include <cxcore.h>
#include <highgui.h>
#include <iostream>
#include <cstdio>

using namespace std;

#include "RobotStatus.h"
#include "RobotInterface.h"

IplImage *statusImg;
CvFont *fontTitles,*fontText;

static int xoff = 10;

static int yimage = 220;
static int yheader = 11;
static int ytext = 20;

static int xwidth = 450;
static int xleft = xoff;
static int xleft2 = xoff+4;
static int textwidth = 60;
static int xright = xwidth-xoff;

static int numinrow = 7;

static int yact;
static int ybinary;
static int ycont;
static int ytotal;


int yoff = 230;
int yoff2 = 370;
int yoff3 = 450;
int yoff4 = 550;

int row,col;
int vspace = 20;
char strval[100];
long int myclock = 0;

void initStatusWindow(){
  statusImg = cvLoadImage(robotImage);

  // Initialize status window
  cvNamedWindow("Robot Status", 1);
  cvShowImage("Robot Status", statusImg);

  // Initialize fonts
  fontTitles = new CvFont();
  fontText = new CvFont();
  cvInitFont(fontTitles, CV_FONT_HERSHEY_TRIPLEX, .4, .4, 0, 1, CV_AA);
  cvInitFont(fontText, CV_FONT_HERSHEY_PLAIN, .7, .7, 0, 1, CV_AA);

  yact = yimage;

  if(numActuators == 0) ybinary = yimage;
  else ybinary = yact + 2*yheader + ytext*((numActuators-1)/numinrow+1)*2;

  if(numBinarySensors == 0) ycont = ybinary;
  else ycont = ybinary + 2*yheader + ytext*((numBinarySensors-1)/numinrow+1)*2;

  if(numContSensors == 0) ytotal = ycont;
  else ytotal = ycont + 2*yheader + ytext*((numContSensors-1)/numinrow+1)*2;

  // Resize the window
  cvResizeWindow("Robot Status", xwidth,ytotal);
  // Create a white rectangle for status info
  cvRectangle(statusImg, cvPoint(xleft,yact),  cvPoint(xright,ytotal), cvScalar(255,255,255), CV_FILLED);

  // Add basic window features
  if(numActuators != 0){
	  cvPutText(statusImg,"Actuators", cvPoint(xleft,yact), fontTitles, cvScalar(0,0,0, 0));
	  cvRectangle(statusImg, cvPoint(xoff,yact+yheader-2), cvPoint(xright,yact+yheader-2), cvScalar(255,0,0), 1);
  }

  if(numBinarySensors != 0){
	  cvPutText(statusImg,"Binary Sensors", cvPoint(xleft,ybinary), fontTitles, cvScalar(0,0,0, 0));
	  cvRectangle(statusImg, cvPoint(xoff,ybinary+yheader-2), cvPoint(xright,ybinary+yheader-2), cvScalar(255,0,0), 1);
  }

  if(numContSensors != 0){
	  cvPutText(statusImg,"Continuous Sensors", cvPoint(xleft,ycont), fontTitles, cvScalar(0,0,0, 0));
	  cvRectangle(statusImg, cvPoint(xoff,ycont+yheader-2), cvPoint(xright,ycont+yheader-2), cvScalar(255,0,0), 1);
  }
}

// This prints all that status information
void updateStatusWindow(){
  if(numActuators != 0){
	  cvRectangle(statusImg, cvPoint(xleft,yact+yheader),  cvPoint(xright,ybinary-yheader), cvScalar(255,255,255), CV_FILLED);
	  for(int i=0; i<numActuators; i++){
		  row = i%7;
		  col = i/7;
		  sprintf((char*)strval,"%d",actuatorValues[i]);
		  cvPutText(statusImg,actuatorNames[i], cvPoint(xleft2+row*textwidth, yact+yheader*2+col*ytext*2), fontText, cvScalar(150, 150, 150, 0));
		  cvPutText(statusImg,strval, cvPoint(xleft2+row*textwidth, yact+yheader*2+ytext+col*(ytext*2)), fontText, cvScalar(0,0,0, 0));
	  }
  }

  if(numBinarySensors != 0){
	  cvRectangle(statusImg, cvPoint(xleft,ybinary+yheader),  cvPoint(xright,ycont-yheader), cvScalar(255,255,255), CV_FILLED);
	  for(int i=0; i<numBinarySensors; i++){
		  row = i%7;
		  col = i/7;
		  int yadd = 25;

		  cvPutText(statusImg,binaryNames[i], cvPoint(xleft2+row*textwidth, ybinary+yheader*2+col*ytext*2), fontText, cvScalar(150, 150, 150, 0));
		  if(binarySensors[i]!=0){
			cvRectangle(statusImg, cvPoint(xleft2+row*textwidth+5, ybinary+yheader*2+5+col*(ytext*2)),  cvPoint(xleft2+(row+1)*textwidth-5, ybinary+yheader*2+ytext+col*(ytext*2)-5), cvScalar(255,0,0), CV_FILLED);
		  }
		  //else
			// cvRectangle(statusImg, cvPoint(xleft2+row*textwidth+5, ybinary+yheader*2+5+col*(ytext*2)),  cvPoint(xleft2+(row+1)*textwidth-5, ybinary+yheader*2+ytext+col*(ytext*2)-5), cvScalar(0,0,0), CV_FILLED);
	  }
  }

  if(numContSensors != 0){
	  cvRectangle(statusImg, cvPoint(xleft,ycont+yheader),  cvPoint(xright,ytotal), cvScalar(255,255,255), CV_FILLED);
	  for(int i=0; i<numContSensors; i++){
		  row = i%7;
		  col = i/7;

		  sprintf((char*)strval,"%4.1d",contSensors[i]);
		  cvPutText(statusImg,contNames[i], cvPoint(xleft2+row*textwidth, ycont+yheader*2+col*ytext*2), fontText, cvScalar(150, 150, 150, 0));
		  cvPutText(statusImg,strval, cvPoint(xleft2+row*textwidth, ycont+yheader*2+ytext+col*(ytext*2)), fontText, cvScalar(0,0,0, 0));
	  }
  }

#if 0
  // Just a test that values change over time (remove it)
  actuatorValues[3]+=1;
  actuatorValues[4]+=2;
  actuatorValues[12]+=1;
  contSensors[5]+=3;
  myclock++;
  //if(myclock%50 == 0) binarySensors[2] = !binarySensors[2];
  // End of test
#endif

  cvShowImage("Robot Status", statusImg);
}



void destroyStatusWindow(){
  cvDestroyWindow("Robot Status");
  cvReleaseImage(&statusImg);
}
