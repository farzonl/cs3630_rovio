#ifndef CAMERA_INTERFACE_H
#define CAMERA_INTERFACE_H

#include <cv.h>
#include <cxcore.h>
#include <highgui.h>

extern IplImage *img;
extern int numCameras;
extern IplImage* images[20];
extern char cameraName[20][1000];
extern char cameraURL[20][50000];

#ifdef _WIN32
void cameraThread(void *arg);
#else
void *cameraThread(void *arg);
#endif
void initCameras();

void processCamera();


#endif
