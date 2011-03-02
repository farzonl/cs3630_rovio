/**
 * @file GaussianBlur.cc
 * @author Noah Kuntz Tutorials
 **/

#include "StdAfx.h"
#include "OpenCVDemo.h"

/** Main Function */
void GaussianBlurDemo(void)
{
   IplImage* img = cvLoadImage("images/Rovio1.jpg");
   cvNamedWindow("Gaussian Blur input");
   cvNamedWindow("Gaussian Blur output");
   cvMoveWindow("Gaussian Blur output", 400,0);
   
   /* Show the original image */
   cvShowImage("Gaussian Blur input",img);
   /** Create an image for the output */
   IplImage* out = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 3);
   /** Perform a Gaussian blur */
   cvSmooth(img, out, CV_GAUSSIAN, 15,15);
   /** Show the processed image */
   cvShowImage("Gaussian Blur output", out);
 
   while(1){
	  char c = cvWaitKey(10);
	  if (c == 27) break;
   }

   cvReleaseImage(&img);
   cvReleaseImage(&out);
   cvDestroyWindow("Gaussian Blur output");
   cvDestroyWindow("Gaussian Blur input");

}
