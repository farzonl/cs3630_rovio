/**
 * @file Threshold.cc
 * @author Noah Kuntz Tutorials 
 */

#include "StdAfx.h"
#include "OpenCVDemo.h"

/** Function headers */
void sum_rgb(IplImage* src, IplImage* dst);

/** Global variables */

/** Main Function */
void ThresholdDemo()
{
   const char* name = "Threshold";
   cvNamedWindow(name, 1);
   
   IplImage* src = cvLoadImage("images/pleo.jpeg");
   IplImage* dst = cvCreateImage(cvGetSize(src), src->depth, 1);
   sum_rgb(src, dst);
   
   cvShowImage(name, dst);
   
   while(1){
      char c = cvWaitKey(15);
      if(c == 27) {break;}
   }
   cvDestroyWindow(name);
   cvReleaseImage(&src);
   cvReleaseImage(&dst);

   return 0;
}


/** Functions*/
void sum_rgb(IplImage* src, IplImage* dst) {
  /** Allocate image planes */
  IplImage *r = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 1);
  IplImage *g = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 1);
  IplImage *b = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 1);
  
  /** Split image onto color planes */
  cvSplit(src, r, g, b, NULL);
  
  IplImage* s = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 1);
  
  /** Add equally weighted rgb values */
  cvAddWeighted(r, 1./3., g, 1./3., 0.0, s);
  cvAddWeighted(s, 2./3., b, 1./3., 0.0, s);
  
  /** Truncate valuese over 100 */
  cvThreshold(s, dst, 100, 100, CV_THRESH_TRUNC);

  cvReleaseImage(&r);
  cvReleaseImage(&g);
  cvReleaseImage(&b);
  cvReleaseImage(&s);  
}
