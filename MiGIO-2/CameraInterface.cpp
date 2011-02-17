#include "CameraInterface.h"
#include "HTTPInterface.h"



#include <stdio.h>
#include <tchar.h>
#include "Blob.h"
#include "BlobResult.h"
#include <fstream>
#ifndef _WIN32
#include <unistd.h>
#endif
using namespace std;

IplImage* img;
int numCameras=0;
IplImage* images[20];
char cameraName[20][1000];
char cameraURL[20][50000];
IplImage* frame=0;
std::vector<CvPoint> path;
int cornersReached = 0;
CvPoint realTopRight, realTopLeft, realBottomLeft, realBottomRight, robotPoint;
CvPoint offset, origin;

int HSV_filter1(int h, int s, int v, int threshold) {
	int FilteredColor[3] = {200, 250, 10}; // This one is Orange HSV
	int diff =  (FilteredColor[0]-h)*(FilteredColor[0]-h) +
				(FilteredColor[1]-s)*(FilteredColor[1]-s);
	
	if(diff < threshold) return abs(diff-threshold); /** If here, it has passed! */
	return 0; /** With 0 this is discarded */
}


void processCamera(){
	IplImage* input;
    IplImage* img;
    IplImage *hsv_img;
    IplImage *bw_img;
    IplImage* i1;
    CBlobResult blobs;
    CBlob blobArea;
	char cname[100];
	sprintf(cname, "Data/Camera0.jpg");
	sprintf(cameraName[numCameras], "Data/Camera0.jpg", numCameras);
	http_fetch(cameraURL[0],cname);
	images[0]=cvLoadImage(cname);
	cvShowImage(cameraName[0], images[0]);

	cvNamedWindow("Blob 1 Demo", CV_WINDOW_AUTOSIZE);

	input = images[0];

	img = cvCloneImage(input);
	hsv_img = cvCloneImage(img);
	bw_img = cvCreateImage(cvSize(img->width, img->height), 8, 1);
    i1 = cvCreateImage(cvSize(img->width, img->height), 8, 1);
    cvZero(i1);
    // Smooth input image using a Gaussian filter, assign HSV, BW image
    cvSmooth(input, img, CV_GAUSSIAN, 7, 9, 0 ,0); 
    cvCvtColor(img, bw_img, CV_RGB2GRAY);	/** Makes image bw_img Blac/White of img  */
    cvCvtColor(img, hsv_img, CV_RGB2HSV);	/** Makes image hsv_img the HSV of img */

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
		

	          blobs = CBlobResult(i1, NULL, 0, true);
          blobs.Filter(blobs, B_INCLUDE, CBlobGetArea(), B_GREATER, 3000);
          blobs.GetNthBlob(CBlobGetArea(), blobs.GetNumBlobs()-1, blobArea); // example
	      printf("The number of blobs is: %d \n", blobs.GetNumBlobs());
          for (int i = 1; i < blobs.GetNumBlobs(); i++ )
             {
	          blobArea = blobs.GetBlob(i);
			  CvBox2D BlobEllipse = blobArea.GetEllipse();
			  CvPoint centrum = cvPoint(BlobEllipse.center.x, BlobEllipse.center.y);
				path.push_back(centrum);

			  if(blobArea.Area() > 300)
	             { blobArea.FillBlob(input, cvScalar(255, 0, 0));
   			       cvCircle(input, centrum, 20, CV_RGB(250, 250, 0),2, 1);}
             }

          // Display blobs
          cvNamedWindow("Input Image - Blob Demo", 1);
          cvShowImage("Input Image - Blob Demo", img);
		  CvScalar cyan = CV_RGB(0, 250,250);
		  CvScalar lightGreen = CV_RGB(127,255,0);
		  CvScalar purple = CV_RGB(138,43,226);
		  
		  // here is where we are going to draw in our squares.
		  if(path.size() > 0){
			
			int xdiff, ydiff;
			if(path.size() > 20){
				CvPoint newest, older;
				newest = path.back();
				older = path.at(path.size()-10);
				xdiff= newest.x - older.x;
				ydiff = newest.y -older.y;
			}
			else{
				xdiff = 0;
				ydiff = 0;
			}
			
			robotPoint = path.back();
			

			int shapeSize = 100;
		  for(int i = 0; i < path.size(); i ++){
			  cvCircle(input, path.at(i),1, cyan, 2);
		  }
			if(1){ //determines if we are going to do (1)square or (0)hourglass.
			  
			  	
			  if(path.size() > 0 && path.size() < 3){
				origin = path.at(0);	
				offset= path.at(0);
				offset.x = offset.x + shapeSize;
				offset.y = offset.y - shapeSize;
				realTopRight = origin;
				realTopLeft = origin;
				realBottomRight = origin;
				realTopRight.x = origin.x + shapeSize;
				realTopRight.y = origin.y - shapeSize;
				realTopLeft.y = origin.y - shapeSize;
				realBottomRight.x = realBottomRight.x + shapeSize;
				}
				cvRectangle(input, origin, offset,lightGreen, 1);
				
				if(cornersReached == 0){
					if((xdiff >15) && (abs(ydiff) <15)){
						cornersReached = 1;
						realTopLeft = path.at(path.size() - 10);
					}
					cvLine(input, origin, robotPoint, purple);
					cvLine(input, robotPoint, realTopLeft, purple);
					cvLine(input, realTopLeft, realTopRight, purple);
					cvLine(input, realTopRight, realBottomRight, purple);
					cvLine(input, realBottomRight, origin, purple);
				}
				else if(cornersReached == 1){
					if((abs(xdiff) <15) && (ydiff >15)){
						cornersReached = 2;
						realTopRight = path.at(path.size() - 10);
					}
					
					cvLine(input, origin, realTopLeft, purple);
					cvLine(input, realTopLeft, robotPoint, purple);
					cvLine(input, robotPoint, realTopRight, purple);
					cvLine(input, realTopRight, realBottomRight, purple);
					cvLine(input, realBottomRight, origin, purple);
				}
				else if(cornersReached == 2){
					if((xdiff < -15) && (abs(ydiff) <15)){
						cornersReached = 3;
						realBottomRight = path.at(path.size() - 10);
					}
					cvLine(input, origin, realTopLeft, purple);
					cvLine(input, realTopLeft, realTopRight, purple);
					cvLine(input, realTopRight, robotPoint, purple);
					cvLine(input, robotPoint, realBottomRight, purple);
					cvLine(input, realBottomRight, origin, purple);
				}
				else if(cornersReached == 3){
					if((abs(origin.x - robotPoint.x) < 15) && (abs(origin.y - robotPoint.y)< 15))
					{
						cornersReached = 4;
					}
					cvLine(input, origin, realTopLeft, purple);
					cvLine(input, realTopLeft, realTopRight, purple);
					cvLine(input, realTopRight, realBottomRight, purple);
					cvLine(input, realBottomRight, robotPoint, purple);
					cvLine(input, robotPoint, origin, purple);
				}
				else{
					cvLine(input, origin, realTopLeft, purple);
					cvLine(input, realTopLeft, realTopRight, purple);
					cvLine(input, realTopRight, realBottomRight, purple);
					cvLine(input, realBottomRight, robotPoint, purple);
				}
			}
			else{
				CvPoint start;
				start = path.at(0);
				CvPoint topLeft = start;
				CvPoint topRight = start;
				CvPoint bottomRight = start;
				topLeft.y = topLeft.y - shapeSize;
				topRight.x = topRight.x + shapeSize;
				topRight.y = topRight.y -shapeSize;
				bottomRight.x = bottomRight.x + shapeSize;
				cvLine(input,start, topRight, cyan, 1);
				cvLine(input,topRight, topLeft, cyan);
				cvLine(input,topLeft, bottomRight, cyan);
				cvLine(input, bottomRight, start, cyan);
			
			
			}
		  }

		  //if(path.size > 1000){
			  
		  //}
	      cvNamedWindow("Output Image - Blob Demo", 1);
          cvShowImage("Output Image - Blob Demo", input);





}

void initCameras(){
	// Initialize all the cameras from infile
	char cname[100];
	fstream infile("Data/config-cam.txt",ios::in);
	while(!infile.eof()){
		infile >> cameraURL[numCameras];
		sprintf(cname, "Data/Camera%d.jpg", numCameras);
		sprintf(cameraName[numCameras], "Data/Camera%d.jpg", numCameras);

		cvNamedWindow(cname);
		http_fetch(cameraURL[numCameras],cname);
		images[numCameras] = cvLoadImage(cname);
		cvShowImage(cameraName[numCameras], images[numCameras]);

		numCameras++;
	}
	infile.close();
}

#ifdef _WIN32
void cameraThread(void *arg)
#else 
void *cameraThread(void *arg)
#endif
{
	while(true){
		for(int i=0; i<numCameras; i++){
			http_fetch(cameraURL[i],cameraName[i]);
			cvReleaseImage(&images[i]);
			images[i] = cvLoadImage(cameraName[i]);
			cvShowImage(cameraName[i], images[i]);
		}
#ifdef _WIN32
		Sleep(100);
#else
		usleep(100);
#endif
	}
}

