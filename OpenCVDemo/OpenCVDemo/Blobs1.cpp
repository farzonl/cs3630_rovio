// blobs.cpp : Defines the entry point for the console application.
//

/*
 *  Blob detector.
 *  Tries to detect an orange blob.
 *  Steps: 1. Create Mask
 *         2. Detect Blobs
 *         3. Filter Blobs
 *  More: Uses OpenCv and cvBlobsLib
 *  Web: 1.
 *       2. http://opencv.willowgarage.com/wiki/cvBlobsLib
 *  Created on: Feb 11, 2010
 *      Author: pushkar
 */
#include "StdAfx.h"
#include "OpenCVDemo.h"

#include "Blob.h"
#include "BlobResult.h"

/**------------------------------*/
/** HSV Simple Filter for colors */
/**------------------------------*/
int HSV_filter1(int h, int s, int v, int threshold) {
	int FilteredColor[3] = {200, 250, 10}; // This one is Orange HSV
	int diff =  (FilteredColor[0]-h)*(FilteredColor[0]-h) +
				(FilteredColor[1]-s)*(FilteredColor[1]-s);
	
	if(diff < threshold) return abs(diff-threshold); /** If here, it has passed! */
	return 0; /** With 0 this is discarded */
}

/**--------------*/
/** Blobs 1 Demo */
/**--------------*/
void Blobs1Demo() {
    IplImage* input;
    IplImage* img;
    IplImage *hsv_img;
    IplImage *bw_img;
    IplImage* i1;
    CBlobResult blobs;
    CBlob blobArea;
	printf("-------------\n");
	printf("Blobs 1 Demo \n");
	printf("-------------\n");

    // Initialize image, allocate memory
   cvNamedWindow("Blob 1 Demo", CV_WINDOW_AUTOSIZE);
   CvCapture* capture = cvCreateFileCapture("video/red.avi");

   while(1) {
		  if(!cvGrabFrame(capture)){              // capture a frame 
			printf("Could not grab a frame stop!\n");
			break;
			}
		  input=cvRetrieveFrame(capture);           // retrieve the captured frame

		  img = cvCloneImage(input);
		  hsv_img = cvCloneImage(img);
		  bw_img = cvCreateImage(cvSize(img->width, img->height), 8, 1);
          i1 = cvCreateImage(cvSize(img->width, img->height), 8, 1);
          cvZero(i1);

          // Smooth input image using a Gaussian filter, assign HSV, BW image
          cvSmooth(input, img, CV_GAUSSIAN, 7, 9, 0 ,0); 
          cvCvtColor(img, bw_img, CV_RGB2GRAY);	/** Makes image bw_img Blac/White of img  */
          cvCvtColor(img, hsv_img, CV_RGB2HSV);	/** Makes image hsv_img the HSV of img */

          // Simple filter that creates a mask whose color is near orange in i1
          for(int i = 0; i < hsv_img->height; i++) {
             for(int j = 0; j < hsv_img->width; j++) {
                 int h = ((uchar *)(hsv_img->imageData + i*hsv_img->widthStep))[j*hsv_img->nChannels + 2];
                 int s = ((uchar *)(hsv_img->imageData + i*hsv_img->widthStep))[j*hsv_img->nChannels + 1];
                 int v = ((uchar *)(hsv_img->imageData + i*hsv_img->widthStep))[j*hsv_img->nChannels + 0];
                 int f = HSV_filter1(h, s, v, 5000);
                 if(f) {
                        ((uchar *)(i1->imageData + i*i1->widthStep))[j] = 255;
                       }
                }
            }

		  // Detect Blobs using the mask and a threshold for area. Sort blobs according to area
          blobs = CBlobResult(i1, NULL, 0, true);
          blobs.Filter(blobs, B_INCLUDE, CBlobGetArea(), B_GREATER, 3000);
          blobs.GetNthBlob(CBlobGetArea(), blobs.GetNumBlobs()-1, blobArea); // example
	      printf("The number of blobs is: %d \n", blobs.GetNumBlobs());
          for (int i = 1; i < blobs.GetNumBlobs(); i++ )
             {
	          blobArea = blobs.GetBlob(i);
			  CvBox2D BlobEllipse = blobArea.GetEllipse();
			  CvPoint centrum = cvPoint(BlobEllipse.center.x, BlobEllipse.center.y);
			  
	          if(blobArea.Area() > 300)
	             { blobArea.FillBlob(input, cvScalar(255, 0, 0));
   			       cvCircle(input, centrum, 20, CV_RGB(250, 250, 0),2, 1);}
             }

          // Display blobs
          cvNamedWindow("Input Image - Blob Demo", 1);
          cvShowImage("Input Image - Blob Demo", img);

	      cvNamedWindow("Output Image - Blob Demo", 1);
          cvShowImage("Output Image - Blob Demo", input);

     	  char c = cvWaitKey(10);
	     if (c == 27) break;
     }
    // Cleanup
    cvReleaseCapture(&capture);
    cvDestroyWindow("Input Image - Blob Demo");
    cvDestroyWindow("Output Image - Blob Demo");
    cvReleaseImage(&i1);
    cvReleaseImage(&bw_img);
    cvReleaseImage(&hsv_img);
    cvReleaseImage(&img);
	cvDestroyWindow("Input Image - Blob Demo");
	cvDestroyWindow("Output Image - Blob Demo");

}

