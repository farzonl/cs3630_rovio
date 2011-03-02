/**
 * @file DownSampleCanny.cc
 * @author Noah Kuntz Tutorials
 * @brief This program reduces the image at half its size and detects edges in it using Canny detection
 **/
#include "StdAfx.h"
#include "OpenCVDemo.h"

/** CannyDemo Function */
void CannyDemo(void)
{
	printf("Canny Edge Detection Demo...\n");
   /** Load the input image as gray (0) */
   IplImage* img = cvLoadImage("images/PleoUp.jpg",0);
   cvNamedWindow("Canny input");
   cvNamedWindow("Canny output");

   /** Move a little bit the output window, so they do not overlap on screen */
   cvMoveWindow("Canny output", 200, 0);
   
   /** Show the original image */
   cvShowImage("Canny input", img);
   

   /** Create an image for the output */
   IplImage* out = cvCreateImage(cvSize(img->width,img->height),img->depth, img->nChannels);
   out = cvLoadImage("images/PleoUp.jpg",0);

   /** Perform canny edge detection */
   cvCanny(out, out, 10,100,3);

   /** Show the processed image */
   cvShowImage("Canny output",out);

   while(1){
	   char c = cvWaitKey(10);
	   if (c == 27) break;
   }

   cvReleaseImage(&img);
   cvReleaseImage(&out);
   cvDestroyWindow("Canny input");
   cvDestroyWindow("Canny output");   
}