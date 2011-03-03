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

#ifndef CVCONSOLE_H
#define CVCONSOLE_H

extern char inputLine[10000];
extern int key;

void initConsole();
void updateConsole();
void destroyConsole();

#endif