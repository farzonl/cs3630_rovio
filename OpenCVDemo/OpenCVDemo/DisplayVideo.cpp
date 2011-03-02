/**
 * @file DisplayVideo.cc    
 * @author Ana
 **/

#include "StdAfx.h"
#include "OpenCVDemo.h"

/** DisplayVideoDemo Function */
void DisplayVideoDemo(void)
{
   cvNamedWindow("Display Video", CV_WINDOW_AUTOSIZE);
   //CvCapture* capture = cvCreateCameraCapture(1);
   CvCapture *capture = cvCreateFileCapture("video/AnDieFreude.mp4");
   IplImage* frame;
   while(1) {
	       frame = cvQueryFrame(capture);
               if(!frame) 
                 { printf(" No captured frame, break\n");
                   break;}
               cvShowImage("Display Video", frame);
               char c = cvWaitKey(33);
               if(c == 27) break;
	    }
    cvReleaseCapture(&capture);
    cvDestroyWindow("Display Video");
}
