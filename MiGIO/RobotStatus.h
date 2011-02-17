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

#ifndef ROBOT_STATUS_H
#define ROBOT_STATUS_H

void initStatusWindow();
void updateStatusWindow();
void destroyStatusWindow();

extern IplImage *statusImg;
extern CvFont *fontTitles,*fontText;

extern char strval[100];

extern long int myclock;

#endif
