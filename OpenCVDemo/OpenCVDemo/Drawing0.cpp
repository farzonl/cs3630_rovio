/**
 * @file Drawing0.cc
 * @author Noah Kuntz Tutorials
 * @brief
 */

#include "StdAfx.h"
#include "OpenCVDemo.h"

/** Drawing0 Demo */
void Drawing0Demo()
{
   IplImage* src1 = cvLoadImage("images/Drawing0.jpg");

   /** Line variables */
   CvPoint pt1 = cvPoint(80,150);
   CvPoint pt2 = cvPoint(450,150);
   int thickness = 2;
   int connectivity = 8;
   CvScalar blue = CV_RGB(0,0,250);
   
   /** Circle variables */
   int radius = 50;
   CvScalar red = CV_RGB(250,0,0);

   /** Text variables */
   const char* text = "Machupicchu - Peru";
   double hscale = 1.0;
   double vscale = 0.8;
   double shear = 0.2;
   int thickness2 = 1;
   int line_type = 8;
   CvScalar purple = CV_RGB(250, 0, 250);

   /** Polygon variables */	
   CvPoint pt3 = cvPoint(10, 25);
   CvPoint pt4 = cvPoint(60, 25);
   CvPoint pt5 = cvPoint(85, 5);
   CvPoint pt6 = cvPoint(110, 25);
   CvPoint pt7 = cvPoint(160, 25);
   CvPoint pt8 = cvPoint(135, 45);
   CvPoint pt9 = cvPoint(160, 65);
   CvPoint pt10 = cvPoint(110, 65);
   CvPoint pt11 = cvPoint(85, 85);
   CvPoint pt12 = cvPoint(60, 65);
   CvPoint pt13 = cvPoint(10, 65);
   CvPoint pt14 = cvPoint(35, 45);


   CvPoint pts_poly[] = {pt3, pt4, pt5, pt6, pt7, pt8, pt9, pt10, pt11, pt12, pt13, pt14};
   CvScalar yellow = CV_RGB(250, 250, 0);

   /** Rectangle variables */
   CvPoint pt15 = cvPoint(400, 20);
   CvPoint pt16 = cvPoint(440, 60);

   CvScalar cyan = CV_RGB(0, 250,250);


   CvFont font1;
   cvInitFont(&font1, CV_FONT_HERSHEY_DUPLEX, hscale, vscale,shear, thickness, line_type);

   cvLine(src1, pt1,pt2, blue, thickness, connectivity);
   cvCircle(src1, pt1, radius, red, thickness, connectivity);
   cvRectangle(src1, pt15, pt16, cyan, -1, connectivity);
   cvPutText(src1, text, pt1, &font1, purple); 
   cvFillConvexPoly(src1, pts_poly, 12, yellow);

   cvNamedWindow("Drawing 0", 1);
   cvShowImage("Drawing 0", src1);
   
      while(1){
	   char c = cvWaitKey(10);
	   if (c == 27) break;
   }

   cvReleaseImage(&src1);
   cvDestroyWindow("Drawing 0");   

}
