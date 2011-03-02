/**
 * @file MouseEvent.cc
 * @author Noah Kuntz Tutorials
 * @brief Detects a mouse event (button up and down) to draw rectangles
 */

#include "StdAfx.h"
#include "OpenCVDemo.h"

/** Functions headers */
void my_mouse_callback(int event, int x, int y, int flags, void* param);
void draw_box(IplImage* img, CvRect rect); 

/** Global variables */
CvRect box;
bool drawing_box = false;

/** Main Fuction */

void MouseEventDemo()
{
   const char* name = "Mouse Event";
   box = cvRect(-1,-1,0,0);
   
   IplImage* image = cvLoadImage("images/cutePleo2.jpeg");
   cvZero(image);
   IplImage* temp = cvCloneImage(image);

   cvNamedWindow(name);
   
   /** Setup the callback */
   cvSetMouseCallback(name, my_mouse_callback, (void*) image);
  
   /** Main Loop */
   while(1) {
       cvCopyImage(image, temp);
       if(drawing_box) 
         { draw_box(temp,box); }
       cvShowImage(name,temp);
       char c = cvWaitKey(15);
       if( c == 27) 
         { break;}
   }

   cvReleaseImage(&image);
   cvReleaseImage(&temp);
   cvDestroyWindow(name);

}


/** Functions implementations */

/** My mouse callback function */
void my_mouse_callback(int event, int x, int y, int flags, void* param)
{
   IplImage* image = (IplImage*) param;
   switch(event) {
      case CV_EVENT_MOUSEMOVE:
	if(drawing_box) {
		box.width = x - box.x;
		box.height = y - box.y;
	}
	break;

      case CV_EVENT_LBUTTONDOWN:
	drawing_box = true;
        box = cvRect( x, y, 0, 0);
        break;
      
      case CV_EVENT_LBUTTONUP:
        drawing_box = false;
        if(box.width  < 0) {box.x += box.width; box.width *= -1;}
        if(box.height < 0) {box.y += box.height; box.height *= -1;}
        draw_box(image, box);
        break;
   }   
}

/** Draw box*/
void draw_box(IplImage* img, CvRect rect)
{
   cvRectangle(img, cvPoint(box.x, box.y), cvPoint(box.x+box.width, box.y+box.height), cvScalar(0xff, 0x00, 0x00));
}

