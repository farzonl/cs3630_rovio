/**
 * @file FaceDetection.cc
 * @author Noah Kuntz Tutorials
 */

#include "StdAfx.h"
#include "OpenCVDemo.h"

void FaceDetectionVideoDemo()
{
   printf("Face Detection Video Demo starting...\n");
   CvCapture* capture = cvCreateFileCapture("video/TheOfficeWedding.mp4");
   IplImage *frame;
   CvMemStorage* storage = cvCreateMemStorage(0);
   CvHaarClassifierCascade* cascade = (CvHaarClassifierCascade*) cvLoad("AdditionalStuff/haarcascade_frontalface_alt2.xml");
   double scale = 1.3;
   CvRect* r;
   static CvScalar colors[] = {{{0,0,255}}, {{0,128,255}},{{0,255,255}},{{0,255,0}},{{255,128,0}},{{255,255,0}},{{255,0,0}},{{255,0,255}}};

  while(1) {
	frame = cvQueryFrame(capture);
        /** Detect objects */
        cvClearMemStorage(storage);
        CvSeq* objects = cvHaarDetectObjects(frame, cascade, storage, 1.1, 4, 0, cvSize(40,50));

       /** Loop through objects and draw  boxes */
       for(int i =0; i<(objects ? objects->total :0); i++) {
		r = (CvRect*) cvGetSeqElem(objects, i);
                cvRectangle(frame, cvPoint(r->x, r->y), cvPoint( r->x + r->width, r->y + r->height), colors[i%8]);  
	}

        if(!frame)
	  { printf("No captured image, break \n");
            break; }
	cvShowImage("Face Detection in Video", frame);
        char c = cvWaitKey(5);
        if( c == 27) break;
	}
  cvReleaseCapture(&capture);
  cvDestroyWindow("Face Detection in Video");
     printf("Face Detection Video Demo ends...\n");
}

