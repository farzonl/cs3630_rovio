/**
 * @file CaptureVideo.cc    
 * @author Ana
 **/

#include "StdAfx.h"
#include "OpenCVDemo.h"

/** CaptureVideoDemo Function */
void CaptureVideoDemo(void)
{
   printf("Capture Video Demo \n");
   cvNamedWindow("Capture", CV_WINDOW_AUTOSIZE);
   CvCapture* capture = cvCaptureFromCAM(0);
   IplImage* frame=0;

   while(1) {
		  frame=cvQueryFrame(capture);          

		  cvShowImage("Capture", frame);
          char c = cvWaitKey(33);
		  if(c == 27) {break;}
	    }
    cvReleaseCapture(&capture);
    cvDestroyWindow("Capture");
}
