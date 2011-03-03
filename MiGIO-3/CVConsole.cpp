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
#include "CVConsole.h"
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include <iostream>
#include <stdio.h>

using namespace std;

char inputLine[10000];
int key;

void initConsole(){
  cout << "Console exists and can be used indirectly from any CV window." << endl;
  cout << "You can handle events like:" << endl;
  cout << " a) Individual characters" << endl;
  cout << " b) Type full lines and process them after the enter key." << endl;
  cout << " c) Both, since backspace works" << endl;

  printf("\n> ");
  strcpy(inputLine,"");
}

void updateConsole(){
	  if(key!=-1 && key !=27 && key!= '\r' && key!='\n'){
		  printf("%c",key);
		  inputLine[strlen(inputLine)] = key;
		  inputLine[strlen(inputLine)+1] = '\0';
	  }else if (key == 27 || key == '\r'){
		  printf(" -- user typed <%s>\n", inputLine);  // Just for testing - REMOVE FOR USE
		  printf("> ");
		  memset(inputLine, 0, 10000);  // Clear user buffer
	  }
}

void destroyConsole(){

}
