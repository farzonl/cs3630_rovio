/**
 * @file RecordVideo.cc    
 * @author Ana
 **/

#include "StdAfx.h"
#include "OpenCVDemo.h"

/** CaptureVideoDemo Function */
void RecordVideoDemo(void)
{

  CvVideoWriter *writer = 0;
  int isColor = 1;
  int fps     = 30;  // or 30 
  int frameW  = 800;  
  int frameH  = 600; 
  writer=cvCreateVideoWriter("IPRDemo.avi",CV_FOURCC('P','I','M','1'), fps,cvSize(frameW,frameH),isColor);

  cvNamedWindow("Captured Stuff");
  CvCapture* capture = cvCaptureFromCAM(0);
  IplImage* img = 0; 
  int nframes = 300;
  for(int i = 0; i < nframes; i++)
  {
     cvGrabFrame(capture);          // capture a frame
     img=cvRetrieveFrame(capture);  // retrieve the captured frame
     cvWriteFrame(writer,img);      // add the frame to the file
  }

  cvReleaseVideoWriter(&writer);
  cvDestroyWindow("Captured Stuff");
}


