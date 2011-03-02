/**
 * @file DisplayImage.cc    
 * @author Noah Kuntz Tutorials 
 **/

#include "StdAfx.h"
#include "OpenCVDemo.h"

/** DisplayImageDemo function */
void DisplayImageDemo()
{
   IplImage* img = 0;
   img = cvLoadImage("images/Llama1.jpg");
   cvNamedWindow("Display Image",CV_WINDOW_AUTOSIZE);
   cvShowImage("Display Image", img);
   cvWaitKey(0);
   cvReleaseImage(&img);
   cvDestroyWindow("Display Image");	
}
