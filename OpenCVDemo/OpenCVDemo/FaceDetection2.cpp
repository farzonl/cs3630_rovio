/**
 * @file FaceDetection.cc
 * @author Noah Kuntz Tutorials
 */
  
#include "StdAfx.h"
#include "OpenCVDemo.h"

void FaceDetection2Demo()
{
   printf("Face Detection Demo \n");
   IplImage *img;
   IplImage *src;

   img = cvLoadImage("images/mikeGolem.jpg");
   src = cvCloneImage(img);

   CvMemStorage* storage = cvCreateMemStorage(0);
   CvHaarClassifierCascade* cascade = (CvHaarClassifierCascade*) cvLoad("AdditionalStuff/haarcascade_frontalface_alt2.xml");
   double scale = 1.3;
   static CvScalar colors[] = {{{0,255,255}}, {{0,128,255}},{{0,255,255}},{{0,255,0}},{{255,128,0}},{{255,255,0}},{{255,0,0}},{{255,0,255}}};
   
   printf("Starting Detection...\n");
   /** Detect objects */
    cvClearMemStorage(storage);
    CvSeq* objects = cvHaarDetectObjects(img, cascade, storage 	, 1.1, 4, 0, cvSize(20,30));

    CvRect* r;
    /** Loop through objects and draw  boxes */
    for(int i =0; i<(objects ? objects->total :0); i++) {
		r = (CvRect*) cvGetSeqElem(objects, i);
                cvRectangle(img, cvPoint(r->x, r->y), cvPoint( r->x + r->width, r->y + r->height), colors[i%8],2);  
	}

	printf("Detection  Finished\n");
    cvNamedWindow("Input Image");
    cvShowImage("Input Image", src);
	cvNamedWindow("Result");
    cvShowImage("Result", img);

    while(1){
	  char c = cvWaitKey(10);
	  if (c == 27) break;
    }

   cvReleaseImage(&img);
   cvReleaseImage(&src);
   cvDestroyWindow("Input Image");
   cvDestroyWindow("Result"); 

}