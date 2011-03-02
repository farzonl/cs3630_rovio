// OpenCVDemo.cpp : Defines the entry point for the console application.
//
#include "StdAfx.h"
#include "OpenCVDemo.h"

/** Function Headers */
void DemoMenu();
void DemoFunction(char key);

/** Main function */
int main(int argc, char* argv[])
{
  DemoMenu();
  return 0;
}

/**
 * @function DemoMenu
 * @brief Shows Menu
 */
void DemoMenu()
{
   /** Text variables */
   const char* textT = "OpenCV Demo - IPR 2011";
   const char* text0 = "<0> Display Image";
   const char* text1 = "<1> Display Video";
   const char* text2 = "<2> Capture Video";
   const char* text3 = "<3> Drawing Basics 0";
   const char* text4 = "<4> Canny Edge Detection";
   const char* text5 = "<5> Gaussian Blur";
   const char* text6 = "<6> Face Detection Demo 2";
   const char* text7 = "<7> Face Detection Video Demo";
   const char* text8 = "<8> Mouse Event Demo";
   const char* text9 = "<9> Filters Demo";
   const char* texta = "<a> Blobs Demo";
   const char* textb = "<b> Blobs Demo Live";
   const char* textc = "<c> Record video";

   CvPoint ptT = cvPoint(50,30);
   CvPoint pt0 = cvPoint(50,45);
   CvPoint pt1 = cvPoint(50,60);
   CvPoint pt2 = cvPoint(50,75);
   CvPoint pt3 = cvPoint(50,90);
   CvPoint pt4 = cvPoint(50,105);
   CvPoint pt5 = cvPoint(50,120);
   CvPoint pt6 = cvPoint(50,135);
   CvPoint pt7 = cvPoint(50,150);
   CvPoint pt8 = cvPoint(50,165);
   CvPoint pt9 = cvPoint(50,180);
   CvPoint pta = cvPoint(50,195);
   CvPoint ptb = cvPoint(50,210);
   CvPoint ptc = cvPoint(50,235);

   double hscale = 0.6;
   double vscale = 0.6;
   double shear = 0.2;
   int thickness = 1;
   int line_type = 8;
   CvScalar purple = CV_RGB(250, 0, 250);

   CvFont font1;
   cvInitFont(&font1, CV_FONT_HERSHEY_DUPLEX, hscale, vscale, shear, thickness, line_type);
   IplImage* src = cvLoadImage("images/Turtle.jpg");
   cvPutText(src, textT, ptT, &font1, purple); 
   cvPutText(src, text0, pt0, &font1, purple); 
   cvPutText(src, text1, pt1, &font1, purple); 
   cvPutText(src, text2, pt2, &font1, purple); 
   cvPutText(src, text3, pt3, &font1, purple);
   cvPutText(src, text4, pt4, &font1, purple);
   cvPutText(src, text5, pt5, &font1, purple);
   cvPutText(src, text6, pt6, &font1, purple);
   cvPutText(src, text7, pt7, &font1, purple);
   cvPutText(src, text8, pt8, &font1, purple);
   cvPutText(src, text9, pt9, &font1, purple);
   cvPutText(src, texta, pta, &font1, purple);
   cvPutText(src, textb, ptb, &font1, purple);
   cvPutText(src, textc, ptc, &font1, purple);

   cvNamedWindow("OpenCV Demo", 1);
   cvShowImage("OpenCV Demo", src);

   while(1) {
       char c = cvWaitKey(15);
       if( c == 'q') 
         { break;}
	   DemoFunction(c);
      }
}


/**
 * @function DemoFunction
 * @brief Choose a function to play
 */
void DemoFunction(char key)
	{
		switch(key)
		{
		case 'q':
			break;
		case 'x':
		    break;
		case '0':
			DisplayImageDemo(); break;
		case '1':
			DisplayVideoDemo(); break;
		case '2':
			CaptureVideoDemo(); break;
		case '3':
			Drawing0Demo();break;
		case '4':
		    CannyDemo();break;
		case '5':
			GaussianBlurDemo(); break;
		case '6':
			FaceDetection2Demo();break;
		case '7':
			FaceDetectionVideoDemo(); break;
		case '8':
			MouseEventDemo(); break;
		case '9':
			FiltersDemo(); break;
		case 'a':
			 Blobs0Demo(); break;
		case 'b':
			Blobs1Demo(); break;
		case 'c':
			RecordVideoDemo(); break;
		case 'd':
			break;
		case 't':
			break;
		default:
			break;
		}
	}