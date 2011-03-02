 /**
 * @file AlphaBlend.cc
 * @author Noah Kuntz Tutorials
 * @brief 
 */

#include "StdAfx.h"
#include "OpenCVDemo.h"

/** AlphaBlendDemo */
void AlphaBlendDemo (void)
{
   /** Load two images */
   IplImage* src1 = cvLoadImage("images/cutePleo1.jpeg");
   IplImage* src2 = cvLoadImage("images/cutePleo2small.jpeg");
   int x = 0;
   int y = 0;
   int width = 102;
   int height = 96;
   double alpha = 0.5;
   double beta = 0.5;
   cvSetImageROI(src1, cvRect(x,y,width,height));
   cvAddWeighted(src1, alpha, src2,beta,0.0,src1);
   cvResetImageROI(src1);
   cvNamedWindow("Alpha blend", 1);
   cvShowImage("Alpha blend", src1);
  
   while(1){
	   char c = cvWaitKey(10);
	   if (c == 27) break;
      }

   cvReleaseImage(&src1);
   cvDestroyWindow("Alpha blend"); 
}
