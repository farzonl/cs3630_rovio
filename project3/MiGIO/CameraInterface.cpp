#include "CameraInterface.h"
#include "HTTPInterface.h"
#include "RobotInterfaceRovio.h"



#include <stdio.h>
//#include <tchar.h>
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
IplImage* background;
IplImage* obstacles;
IplImage* ObstacleBackground;
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
int first = 1;

void setBackground(){
	int key;
	IplImage* img;
    
	while(1){
        key = cvWaitKey(30);
        char cname[100];
        sprintf(cname, "Data/Camera0.jpg");
        sprintf(cameraName[numCameras], "Data/Camera0.jpg");
        http_fetch(cameraURL[0],cname);
        //	cvReleaseImage(&images[0]);
        img=cvLoadImage(cname);
        cvShowImage("background" , img);
        if(key == '1')
            break;
	}
	cvSmooth(img, img, CV_GAUSSIAN);
	background = cvCloneImage(img);
	//cvReleaseImage( &images[0]);
}

void setObstacleBackground(){
    int key, backr, backg, backb;
	while(1){
        key = cvWaitKey(30);
        char cname[100];
        sprintf(cname, "Data/Camera0.jpg");
        sprintf(cameraName[numCameras], "Data/Camera0.jpg");
        http_fetch(cameraURL[0],cname);
        //	cvReleaseImage(&images[0]);
        images[0]=cvLoadImage(cname);
        cvShowImage("background with obstacles" , images[0]);
        if(key == '2')
            break;
	}
    IplImage* img;
    
    //	    cvSmooth(img, img, CV_GAUSSIAN); 
    
	img = images[0];
	cvSmooth(img, img, CV_GAUSSIAN);
	ObstacleBackground = cvCloneImage(img);
	//cvCopyImage(img, ObstacleBackground);
	//cvReleaseImage( &images[0]);
    
	for(int i = 0; i < img->height; i++) {
		for(int j = 0; j < img->width; j++) {
			int r = ((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 2];
            int g = ((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 1];
            int b = ((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 0];
			backr = ((uchar *)(background->imageData + i*img->widthStep))[j*img->nChannels + 2];
			backg = ((uchar *)(background->imageData + i*img->widthStep))[j*img->nChannels + 1];
			backb = ((uchar *)(background->imageData + i*img->widthStep))[j*img->nChannels + 0];
			if(((r-backr)*(r-backr) + (g-backg)*(g-backg)+ (b-backb)*(b-backb)) < 1000){
				((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 2] = 0;
                
				((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 1] = 0;
				((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 0] = 0;
			}
			else{
				((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 2] = 255;
				((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 1] = 0;
				((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 0] = 0;
			}
		}
	}
    //	cvShowImage("obstacles", img);
	obstacles = cvCloneImage(img);
	cvShowImage("obstacles", obstacles);
    
}

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
	int FilteredColor[3] = {10, 110, 60};//the filter for green
	int diff = (FilteredColor[0] - r)*(FilteredColor[0]-r)+
    (FilteredColor[1] - g)*(FilteredColor[1] - g)+
    (FilteredColor[2] - b)*(FilteredColor[2] - b);
	//printf("out");
	if(diff < threshold) return abs(diff - threshold);
	return 0;
}

int RGB_filter2(int r, int g, int b, int threshold){
	int FilteredColor[3] = {40, 60, 160}; //the RGB values for the blue felt.
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
	//int FilteredColor[3] = {240, 70, 120}; //the RGB values for teh apple.
	int FilteredColor[3] = {255, 250, 240}; // the RGB values for teh lemon
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
    IplImage* i1;
	IplImage* rob;
	IplImage* fruit;
	CBlobResult robBlobs;
    CvPoint location;
	CBlobResult fruitBlob;
	CBlob fruitBlobArea;
	CBlob robBlobArea;
    CBlobResult blobs;
    CBlob blobArea;
	if(first){
        first = 0;
        setBackground();
        setObstacleBackground();
	}
	char cname[100];
	sprintf(cname, "Data/Camera0.jpg");
	sprintf(cameraName[numCameras], "Data/Camera0.jpg");
	http_fetch(cameraURL[0],cname);
    //	cvReleaseImage(&images[0]);
	images[0]=cvLoadImage(cname);
	cvShowImage(cameraName[0], images[0]);
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
    cvSmooth(input, img, CV_GAUSSIAN); 
    
	for(int i = 0; i < img->height; i ++){
		for(int j = 0; j < img->width; j++){
			int r,g,b, backr, backg, backb;
			r = ((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 2];
            g = ((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 1];
            b = ((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 0];
			backr = ((uchar *)(ObstacleBackground->imageData + i*ObstacleBackground->widthStep))[j*img->nChannels + 2];
			backg = ((uchar *)(ObstacleBackground->imageData + i*ObstacleBackground->widthStep))[j*img->nChannels + 1];
			backb= ((uchar *)(ObstacleBackground->imageData + i*ObstacleBackground->widthStep))[j*img->nChannels + 0];
			if(((r-backr)*(r-backr) + (g-backg)*(g-backg)+ (b-backb)*(b-backb)) < 1000){
				((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 2] = 0;
				((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 1] = 0;
				((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 0] = 0;
			}
		}
	}
    
	cvShowImage("Background subtracted", img);
	for(int i = 0; i < img->height; i++) {
		for(int j = 0; j < img->width; j++) {
			int r = ((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 2];
            int g = ((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 1];
            int b = ((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 0];
            int f = RGB_filter1(r, g, b, 6000);
            if(f) {
				((uchar *)(i1->imageData + i*i1->widthStep))[j] = 255;
            }
            
			int h = RGB_filter2(r, g, b, 6000);
			if(h) {
				//printf("test");q
                
				((uchar *)(rob->imageData + i*rob->widthStep))[j] = 255;
            }
			int fr = RGB_filter3(r,g,b, 6000);
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
    fruitBlob.Filter(fruitBlob, B_INCLUDE, CBlobGetArea(), B_GREATER, 10);
    fruitBlob.Filter(fruitBlob, B_INCLUDE, CBlobGetArea(), B_LESS, 6000);
    blobs.Filter(blobs, B_INCLUDE, CBlobGetArea(), B_GREATER, 50);
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
        cvCircle(input, Right, 20, CV_RGB(250, 250, 250),2,1);
        cvCircle(input, Left, 20, CV_RGB(127, 127, 127), 2, 1);
        cvCircle(input, target, 20, CV_RGB(0, 0, 0), 2, 1);
        double temp = sqrt((double)(Right.x - Left.x)*(Right.x - Left.x) + (Right.y - Left.y)*(Right.y - Left.y));
        temp = acos((Right.x-Left.x)/temp);
        temp = (temp/3.14159)*180;
        //temp = temp + 90;
        //printf("angle %d ", temp);
        if(((Right.y - Left.y) < 0) && ((Right.x-Left.x) > 0)){
            //printf("ugh");
            temp = 360 -temp;
        }
        else if(((Right.y - Left.y < 0) && ((Right.x - Left.x) < 0))){
            // printf("oop");
            float rho = 180 - temp;
            temp = temp + 2*rho;
        }
        temp = temp + 90;
        temp = (int)temp%360;
        if(temp > 180){
            temp = (180 + 360 - temp);
        }
        if(temp < 180){
            temp = 180 - temp;
        }
        orientation = temp;
        
        location.x = (int)(Right.x + Left.x)/2;
        location.y = (int)(Right.y + Left.y)/2;
        double fruitAngle = sqrt((double)(target.x - location.x)*(target.x - location.x) +
                                 (target.y - location.y)*(target.y-location.y));
        fruitAngle = acos((target.x - location.x)/fruitAngle);
        fruitAngle = (fruitAngle/3.14159)*180;
        if(((target.y - location.y) > 0) && ((target.x-location.x) > 0)){
            //printf("ugh");
            fruitAngle = 360 -fruitAngle;
        }
        else if(((target.y - location.y > 0) && ((target.x - location.x) < 0))){
            //printf("oop");
            float rho = 180 - fruitAngle;
            fruitAngle = fruitAngle + 2*rho;
        }
        
        //printf("fruit angle: %d", (int)fruitAngle);
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
        
        printf("The orientation is: %d \n", difference);
    }
 

    blobs.ClearBlobs();
    cvReleaseImage(&img);

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
    }
 
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

