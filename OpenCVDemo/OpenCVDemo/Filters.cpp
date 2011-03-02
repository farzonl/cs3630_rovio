/**
 * @file Filters.cc
 * @author Noah Kuntz Tutorials
 */

#include "StdAfx.h"
#include "OpenCVDemo.h"

/** Global Variables */
int g_switch_value  = 0;
int filterInt = 0;
int lastfilterInt = -1;

void switch_callback(int position){
  filterInt = position;
}

/** Main Function */

void FiltersDemo()
{
   const char* name = "Filters";
   IplImage* img = cvLoadImage("images/cutePleo2.jpeg");
   IplImage* out = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 3);
   
   cvNamedWindow(name, 1);
   cvShowImage(name,out);

   /* Other variables */
   CvPoint seed_point = cvPoint(100, 100);
   CvScalar color = CV_RGB(250,0,0);
   
  /* Create trackbar */
  cvCreateTrackbar("Filter type", name, &g_switch_value, 5, switch_callback);

  while(1) {
	switch(filterInt) {
	   case 0:
		cvSmooth(img, out, CV_BLUR, 7,7); break;
	   case 1:
 		cvSmooth(img, out, CV_GAUSSIAN, 7,7); break;
	   case 2:
		cvSmooth(img, out, CV_MEDIAN, 7,7); break;
	   case 3:
		cvErode(img, out, NULL, 1); break;
	   case 4: 
	        cvDilate(img, out, NULL, 1); break;
           case 5:
	        cvFloodFill(out, seed_point, color, cvScalarAll(5.0), cvScalarAll(5.0), NULL, 4, NULL); break;
	}
	if(filterInt != lastfilterInt) {
		cvShowImage(name, out);
                lastfilterInt = filterInt;
		}
	char c = cvWaitKey(15);
        if(c == 27) break;
  }

  cvReleaseImage(&img);
  cvReleaseImage(&out);
  cvDestroyWindow(name);

}