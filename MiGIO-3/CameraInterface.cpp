#include "CameraInterface.h"
#include "HTTPInterface.h"
#include "RobotInterfaceRovio.h"



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
std::vector<CvPoint> robotPos;
int cornersReached = 0;
CvPoint realTopRight, realTopLeft, realBottomLeft, realBottomRight, robotPoint;
CvPoint offset, origin;
int imageCount =0;
int localCount = 0;
CvPoint midFront;
CvPoint midBack;
CvPoint Left, Right;
int orientation;
CvPoint target;

int HSV_filter1(int h, int s, int v, int threshold) {
//	printf("H: %d S: %d V: %d \n", h, s, v);
//	int FilteredColor[3] = {200, 250, 10}; // This one is Orange HSV
	int FilteredColor[3] = {200, 20,200}; //this works for pink, but not white.
	int diff =  (FilteredColor[0]-h)*(FilteredColor[0]-h) +
				(FilteredColor[1]-s)*(FilteredColor[1]-s) + 
				(FilteredColor[2]-v)*(FilteredColor[2]-v);
	
	if(diff < threshold) return abs(diff-threshold); /** If here, it has passed! */
	return 0; /** With 0 this is discarded */
}

int RGB_filter1(int r, int g, int b, int threshold){
	//int FilteredColor[3] = {190, 190, 75}; //the RGB values for bright yellow.
	int FilteredColor[3] = {30, 140, 90};//the filter for green
	int diff = (FilteredColor[0] - r)*(FilteredColor[0]-r)+
				(FilteredColor[1] - g)*(FilteredColor[1] - g)+
				(FilteredColor[2] - b)*(FilteredColor[2] - b);
	//printf("out");
	if(diff < threshold) return abs(diff - threshold);
	return 0;
}

int RGB_filter2(int r, int g, int b, int threshold){
	int FilteredColor[3] = {90, 90, 200}; //the RGB values for the blue felt.
	//if((r > 150)&&(g > 150) && (b > 150) && ( b > 175) && ( b > r + 20) && ( b > g +40)){
	//	return 1;
	//}
	int diff = (FilteredColor[0] - r)*(FilteredColor[0]-r)+
				(FilteredColor[1] - g)*(FilteredColor[1] - g)+
				(FilteredColor[2] - b)*(FilteredColor[2] - b);
	//printf("out");
	if(diff < threshold){
		//printf("passed");
		return abs(diff - threshold);
		
	}
	return 0;
}

int RGB_filter3(int r, int g, int b, int threshold){
	int FilteredColor[3] = {230, 100, 100}; //the RGB values for teh lemon.
	int diff = (FilteredColor[0] - r)*(FilteredColor[0]-r)+
				(FilteredColor[1] - g)*(FilteredColor[1] - g)+
				(FilteredColor[2] - b)*(FilteredColor[2] - b);
	//printf("out");
	if(diff < threshold){
//		printf("passed");
		return abs(diff - threshold);
	}
	return 0;
}


void processCamera(){
	IplImage* input;
    IplImage* img;
//    IplImage *hsv_img;
//    IplImage *bw_img;
    IplImage* i1;
	IplImage* rob;
	IplImage* fruit;
	CBlobResult robBlobs;
	CBlobResult fruitBlob;
	CBlob fruitBlobArea;
	CBlob robBlobArea;
    CBlobResult blobs;
    CBlob blobArea;
	char cname[100];
	sprintf(cname, "Data/Camera0.jpg");
	sprintf(cameraName[numCameras], "Data/Camera0.jpg", numCameras);
	http_fetch(cameraURL[0],cname);
//	cvReleaseImage(&images[0]);
	images[0]=cvLoadImage(cname);
	cvShowImage(cameraName[0], images[0]);

	//cvNamedWindow("Blob 1 Demo", CV_WINDOW_AUTOSIZE);

	input = images[0];

	img = cvCloneImage(input);
	//hsv_img = cvCloneImage(img);
	//bw_img = cvCreateImage(cvSize(img->width, img->height), 8, 1);
	rob = cvCreateImage(cvSize(img->width, img->height), 8,1);
	fruit = cvCreateImage(cvSize(img->width, img->height), 8, 1);
	i1 = cvCreateImage(cvSize(img->width, img->height), 8, 1);
    cvZero(i1);
	cvZero(rob);
	cvZero(fruit);
    // Smooth input image using a Gaussian filter, assign HSV, BW image
    cvSmooth(input, img, CV_GAUSSIAN, 7, 9, 0 ,0); 
    //cvCvtColor(img, bw_img, CV_RGB2GRAY);	/** Makes image bw_img Blac/White of img  */
    //cvCvtColor(img, hsv_img, CV_RGB2HSV);	/** Makes image hsv_img the HSV of img */

	for(int i = 0; i < img->height; i++) {
		for(int j = 0; j < img->width; j++) {
			int r = ((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 2];
            int g = ((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 1];
            int b = ((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 0];
            int f = RGB_filter1(r, g, b, 5000);
            if(f) {
				((uchar *)(i1->imageData + i*i1->widthStep))[j] = 255;
                }
            
			int h = RGB_filter2(r, g, b, 5000);
			if(h) {
				//printf("test");

				((uchar *)(rob->imageData + i*rob->widthStep))[j] = 255;
				}
			int fr = RGB_filter3(r,g,b, 4000);
			if(fr){
				//printf("passed:");
				((uchar *)(fruit->imageData + i*fruit->widthStep))[j] = 255;
			}
			}
		}
		int both = 0;
			robBlobs = CBlobResult(rob, NULL, 0, true);
	          blobs = CBlobResult(i1, NULL, 0, true);
			  fruitBlob = CBlobResult(fruit, NULL, 0, true);
			  //cvReleaseImage(&i1); 
			  fruitBlob.Filter(fruitBlob, B_INCLUDE, CBlobGetArea(), B_GREATER, 50);
			  fruitBlob.Filter(fruitBlob, B_INCLUDE, CBlobGetArea(), B_LESS, 6000);
          blobs.Filter(blobs, B_INCLUDE, CBlobGetArea(), B_GREATER, 200);
		  blobs.Filter(blobs, B_INCLUDE, CBlobGetArea(), B_LESS, 6000);
		  robBlobs.Filter(robBlobs, B_INCLUDE, CBlobGetArea(), B_GREATER, 20);
		  //robBlobs.Filter(robBlobs, B_INCLUDE, CBlobGetArea(), B_LESS, 6000);
          //blobs.GetNthBlob(CBlobGetArea(), blobs.GetNumBlobs()-1, blobArea); // example
	      //printf("The number of blobs is: %d \n", blobs.GetNumBlobs());
          for (int i = 1; i < blobs.GetNumBlobs(); i++ )
             {
				//printf("blobbled");
	          blobArea = blobs.GetBlob(i);
			  CvBox2D BlobEllipse = blobArea.GetEllipse();
			  CvPoint centrum = cvPoint(BlobEllipse.center.x, BlobEllipse.center.y);
			//	if((centrum.x != 0) &&( centrum.y != 0)){
			//  path.push_back(centrum);
			//	}
			  Right = centrum;
			  if(both == 0){
				  both = 1;
			  }
			  if(blobArea.Area() > 10)
	             { blobArea.FillBlob(input, cvScalar(255, 0, 0));
   			       cvCircle(input, centrum, 20, CV_RGB(250, 250, 0),2, 1);}
             }
		  //printf("The number of fruit blobs is: %d \n", fruitBlob.GetNumBlobs());
		  
		  for (int i = 1; i < fruitBlob.GetNumBlobs(); i++ )
             {
				//printf("blobbled");
	          fruitBlobArea = fruitBlob.GetBlob(i);
			  CvBox2D BlobEllipse = fruitBlobArea.GetEllipse();
			  CvPoint centrum = cvPoint(BlobEllipse.center.x, BlobEllipse.center.y);
				//if((centrum.x != 0) &&( centrum.y != 0)){
			 // path.push_back(centrum);
				//}
			  target = centrum;
			  if(fruitBlobArea.Area() > 10)
	             { fruitBlobArea.FillBlob(input, cvScalar(255, 0, 250));
   			       cvCircle(input, centrum, 20, CV_RGB(250, 0, 250),2, 1);}
             }
		  
		  //printf("The number of robot blobs is: %d \n", robBlobs.GetNumBlobs());
		  
		  robotPos.clear();
		  for (int i = 1; i < robBlobs.GetNumBlobs(); i++ )
             {
				//printf("blobbled");
	          robBlobArea = robBlobs.GetBlob(i);
			  CvBox2D BlobEllipse = robBlobArea.GetEllipse();
			  CvPoint centrum = cvPoint(BlobEllipse.center.x, BlobEllipse.center.y);
			  	if((centrum.x != 0) &&( centrum.y != 0)){
					//int passes = 1;
					//for(int k = 0; k < robotPos.size(); k ++ ){
					//	int passes = 1;
					//	double TempDist = sqrt((double)((centrum.x - robotPos.at(k).x)*(centrum.x - robotPos.at(k).x))
					//		+ ((centrum.y - robotPos.at(k).y)*(centrum.y - robotPos.at(k).y)));
					//	if(TempDist < 10){
					//	 passes = false;
					//	}
					//}
					//robotPos.push_back(centrum);
				}
				Left = centrum;
				if(both == 1){
					both = 2;
				}

			  if(robBlobArea.Area() > 4)
	             { robBlobArea.FillBlob(input, cvScalar(0, 255, 0));
   			       cvCircle(input, centrum, 20, CV_RGB(250, 250, 0),2, 1);}
             }

		  if(both ==2){
			  double temp = sqrt((double)(Right.x - Left.x)*(Right.x - Left.x) + (Right.y - Left.y)*(Right.y - Left.y));
				  temp = acos((Right.x-Left.x)/temp);
			  temp = (temp/3.14159)*180;
  			  orientation = temp + 90;
			  CvPoint location;
			  location.x = (int)(Right.x + Left.x)/2;
			  location.y = (int)(Right.y + Left.y)/2;
			  double fruitAngle = sqrt((double)(target.x - location.x)*(target.x - location.x) +
					(target.y - location.y)*(target.y-location.y));
			  fruitAngle = acos((target.x - location.x)/fruitAngle);
			  fruitAngle = (fruitAngle/3.14159)*180;
			  int difference = (fruitAngle - orientation);
			  if(difference > 180){
				  difference = difference - 360;
			  }
			  if(difference < -180){
				  difference = difference + 360;
			  }
			  CvPoint relativeFruitPos;
			  relativeFruitPos.x = (int)cos((double)((difference/360)*3.14159)) * 8;
			  relativeFruitPos.y = (int)sin((double)((difference/360)*3.14159)) * 8;
			  

			 // if((difference > 30)){
			//	  turnLeft(difference);
			//  }
			//  else if((difference < -30)){
			//	  turnRight(difference);
			//  }
			  //forward();
			  printf("The orientation is: %d \n", difference);
		  }
		 // printf("The number of robot blobs is: %d \n", robotPos.size());
		  // Here is where we will detect the position an orientation of the robot from the detected lights on its back.
		  //printf("erg");
		 /* if((robotPos.size() >= 4)&& (robotPos.size() <= 6 )){
			  CvPoint positions[6];
			  for(int i = 0; i < robotPos.size(); i ++){
				  positions[i] = robotPos.at(i);
			  }
			  CvPoint maxPair;
			  maxPair.x = 0;
			  maxPair.y = 0;
			  CvPoint secondMaxPair;
			  //printf("HASDFHASDF");
			float maxDist = 0;
			float secondMaxDist = 0;
			for(int i = 0; i < robotPos.size(); i ++){
				for(int j = i+1; j < robotPos.size(); j++){
					
					float dist = sqrt((float)(positions[i].x -  positions[j].x)*(positions[i].x - positions[j].x)
						 + (float)(positions[i].y - positions[j].y)*(positions[i].y - positions[i].y));
					if(dist > secondMaxDist){
						secondMaxDist = dist;
						secondMaxPair.x = i;
						secondMaxPair.y = j;
					}
					if(dist > maxDist){
						secondMaxDist = maxDist;
						secondMaxPair = maxPair;
						maxDist = dist;
						maxPair.x = i;
						maxPair.y = j;
					}

				}
		  }
			secondMaxDist = 0;
			for(int i = 0; i < robotPos.size(); i ++){
				for(int j = i+1; j < robotPos.size(); j++){
					if((i == maxPair.x) || (j == maxPair.y) || (i == maxPair.y) || (j == maxPair.x)){
						continue;
					}
					float dist = sqrt((float)(positions[i].x -  positions[j].x)*(positions[i].x - positions[j].x)
						 + (float)(positions[i].y - positions[j].y)*(positions[i].y - positions[i].y));
					if(dist > secondMaxDist){
						secondMaxDist = dist;
						secondMaxPair.x = i;
						secondMaxPair.y = j;
					}
					if(dist > maxDist){
						secondMaxDist = maxDist;
						secondMaxPair = maxPair;
						maxDist = dist;
						maxPair.x = i;
						maxPair.y = j;
					}

				}
		  }
			cvLine(input, positions[maxPair.x], positions[maxPair.y], CV_RGB(138,43,226));
			cvLine(input, positions[secondMaxPair.x], positions[secondMaxPair.y], CV_RGB(138,43,226));
			//now, using the four points that are the furthest apart. We need to find the two points closest to each other
			// and the two furthest from each other.
			float dist1 = sqrt((float)(positions[maxPair.x].x - positions[secondMaxPair.x].x)*
				(positions[maxPair.x].x - positions[secondMaxPair.x].x) 
				+ ((positions[maxPair.x].y - positions[secondMaxPair.x].y) 
				* (positions[maxPair.x].y - positions[secondMaxPair.x].y)));
			float dist2 = sqrt((float)(positions[maxPair.x].x - positions[secondMaxPair.y].x)*
				(positions[maxPair.x].x - positions[secondMaxPair.y].x) 
				+ ((positions[maxPair.x].y - positions[secondMaxPair.y].y) 
				* (positions[maxPair.x].y - positions[secondMaxPair.y].y)));
			float dist3 = sqrt((float)(positions[maxPair.y].x - positions[secondMaxPair.x].x)*
				(positions[maxPair.y].x - positions[secondMaxPair.x].x) 
				+ ((positions[maxPair.y].y - positions[secondMaxPair.x].y) 
				* (positions[maxPair.y].y - positions[secondMaxPair.x].y)));
			float dist4= sqrt((float)(positions[maxPair.y].x - positions[secondMaxPair.y].x)*
				(positions[maxPair.y].x - positions[secondMaxPair.y].x) 
				+ ((positions[maxPair.y].y - positions[secondMaxPair.y].y) 
				* (positions[maxPair.y].y - positions[secondMaxPair.y].y)));
		  float minDist = dist1;
		  int min = 1;
		  if(dist2 < minDist){
			minDist = dist2; 
			min = 2;
		  }
		  if(dist3 < minDist){
			  minDist = dist3;
			  min = 3;
		  }
		  if(dist4 < minDist){
			  minDist = dist4;
			  min = 4;
		  }
		  CvPoint frontPair, backPair;
			
		  if(min == 1){
				frontPair.x = maxPair.x;
				frontPair.y = secondMaxPair.x;
				backPair.x = maxPair.y;
				backPair.y = secondMaxPair.y;
		  }
		  else if(min == 2){
				frontPair.x = maxPair.x;
				frontPair.y = secondMaxPair.y;
				backPair.x = maxPair.y;
				backPair.y = secondMaxPair.x;
		  }
		  else if(min == 3){
				frontPair.x = maxPair.y;
				frontPair.y = secondMaxPair.x;
				backPair.x = maxPair.x;
				backPair.y = secondMaxPair.y;
		  }
		  else{
			  	frontPair.x = maxPair.y;
				frontPair.y = secondMaxPair.y;
				backPair.x = maxPair.x;
				backPair.y = secondMaxPair.x;
		  }

		  midFront.x = positions[frontPair.x].x + (.5*(positions[frontPair.x].x - positions[frontPair.y].x));
		  midFront.y = positions[frontPair.x].y + (.5*(positions[frontPair.x].y - positions[frontPair.y].y));
		  midBack.x = positions[backPair.x].x + (.5*(positions[backPair.x].x - positions[backPair.y].x));
		  midBack.y = positions[backPair.x].y + (.5*(positions[backPair.x].y - positions[backPair.y].y));	
		  cvLine(input, midBack, midFront, CV_RGB(138,43,226));
		  //printf("yeah, bi-atches");
		  //at this point, midFront is the front center point of the robot, midBack is the back point. 
		  //The orientation is based on the angle based on the path from midBack to MidFront.
		  }*/


          // Display blobs
//          cvNamedWindow("Input Image - Blob Demo", 1);
 //         cvShowImage("Input Image - Blob Demo", img);
		  blobs.ClearBlobs();
		  cvReleaseImage(&img);
//		  cvReleaseImage(&hsv_img);
//		  cvReleaseImage(&bw_img);
		  cvReleaseImage(&fruit);
		  cvReleaseImage(&rob);
		  robBlobs.ClearBlobs();
		  cvReleaseImage(&i1);
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
	/*		if(0){ //determines if we are going to do (1)square or (0)hourglass.
			  
			  	
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
				if(path.size() == 1){
					origin = path.at(0);
					realTopLeft = origin;
					realTopRight = origin;
					realBottomRight = origin;
					realTopLeft.y = realTopLeft.y - shapeSize;
					realTopRight.y = realTopRight.y - shapeSize;
					realTopRight.x = realTopRight.x + shapeSize;
					realBottomRight.x = realBottomRight.x + shapeSize;
				}
				cvLine(input, origin, realTopRight, lightGreen);
				cvLine(input, realTopRight, realTopLeft, lightGreen);
				cvLine(input, realTopLeft, realBottomRight, lightGreen);
				cvLine(input, realBottomRight, origin, lightGreen);
				
				if(cornersReached == 0){
					if((xdiff < -15) && (abs(ydiff) < 15)){
						cornersReached = 1;
						realTopRight = path.at(path.size()-10);
					}
					cvLine(input, origin, robotPoint, purple);
					cvLine(input, robotPoint, realTopRight, purple);
					cvLine(input, realTopRight, realTopLeft, purple);
					cvLine(input, realTopLeft, realBottomRight, purple);
					cvLine(input, realBottomRight, origin, purple);
				}
				else if(cornersReached == 1){
					if((xdiff > 15) && (ydiff > 15)){
						cornersReached = 2;
						realTopLeft = path.at(path.size()-10);
					}
					cvLine(input, origin, realTopRight, purple);
					cvLine(input, realTopRight, robotPoint, purple);
					cvLine(input, robotPoint, realTopLeft, purple);
					cvLine(input, realTopLeft, realBottomRight, purple);
					cvLine(input, realBottomRight, origin, purple);

				}
				else if(cornersReached == 2){
					if((xdiff < -15) && (abs(ydiff) < 15)){
						cornersReached = 3;
						realBottomRight = path.at(path.size()-10);
					}
					cvLine(input, origin, realTopRight, purple);
					cvLine(input, realTopRight, realTopLeft, purple);
					cvLine(input, realTopLeft, robotPoint, purple);
					cvLine(input, robotPoint, realBottomRight, purple);
					cvLine(input, realBottomRight, origin, purple); 
				}
				else if(cornersReached == 3){
					if((abs(robotPoint.x - origin.x) < 15) && (abs(robotPoint.y - origin.y) < 15)){
						cornersReached = 4;
					}
					cvLine(input, origin, realTopRight, purple);
					cvLine(input, realTopRight, realTopLeft, purple);
					cvLine(input, realTopLeft, realBottomRight, purple);
					cvLine(input, realBottomRight, robotPoint, purple);
					cvLine(input, robotPoint, origin, purple);
				}
				else{
					cvLine(input, origin, realTopRight, purple);
					cvLine(input, realTopRight, realTopLeft, purple);
					cvLine(input, realTopLeft, realBottomRight, purple);
					cvLine(input, realBottomRight, robotPoint, purple);
				}
				
				
				//start = path.at(0);
				//CvPoint topLeft = start;
				//CvPoint topRight = start;
				//CvPoint bottomRight = start;
				//topLeft.y = topLeft.y - shapeSize;
				//topRight.x = topRight.x + shapeSize;
				//topRight.y = topRight.y -shapeSize;
				//bottomRight.x = bottomRight.x + shapeSize;
				//cvLine(input,start, topRight, cyan, 1);
				//cvLine(input,topRight, topLeft, cyan);
				//cvLine(input,topLeft, bottomRight, cyan);
				//cvLine(input, bottomRight, start, cyan);
			
			
			}*/
		  }

		  //if(path.size > 1000){
			  
		  //}
	      cvNamedWindow("Output Image - Blob Demo", 1);
		  if(localCount % 5 ==0){
			  localCount = 0;
			  string imageName;
			  std::stringstream out;
			  out << "trace" << imageCount <<".jpg";
			  imageName = out.str();
			  cvSaveImage(imageName.c_str(), input);
			  imageCount= imageCount + 1;
		  }
		  localCount++;
          cvShowImage("Output Image - Blob Demo", input);
		  cvReleaseImage(&input);

		



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

