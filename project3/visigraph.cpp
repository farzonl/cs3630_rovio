#include <cv.h>
#include <highgui.h>
#include "visilibity.hpp"
#include "blob.h"
#include "BlobResult.h"

/* 
 steps
 
 - creation of map
 <creation of obstacle image goes here from video>
 1. load obstacle image from disk (hack)
 2. run blob detection on red things
 3. create corner image
 4. (however this works) mark corners inside blobs, create polygons with that
 
 5. create visibility environment with empty space = everything not an obstacle
 
 6. get a graph or something
 
 - motion along map
 */

CvMemStorage *gStorage = NULL;
static std::vector<CvBox2D> obstacleboxes;

static IplImage *same_size_image_8bit(IplImage *frame)
{
    return cvCreateImage(cvSize(frame->width, frame->height), IPL_DEPTH_8U, 1);
}

static void draw_rect(IplImage *frame, CvRect r, int color)
{
    // drawing
    CvPoint pt1, pt2;
    const CvScalar colors[] = {
        CV_RGB(255, 0, 0), // face red
        CV_RGB(0, 0, 255), // blob blue
        CV_RGB(255, 255, 255), // chosen white
    };
        
    pt1.x = r.x;
    pt2.x = r.x + r.width;
    pt1.y = r.y;
    pt2.y = r.y + r.height;
    
    //printf("rect: x %d y %d w %d h %d\n", r.x, r.y, r.width, r.height);
    
    cvRectangle(frame, pt1, pt2, colors[color], 3, 8, 0);
}

// returns input image (obstacled-highlighted)
// with the visibility graph drawn on top
static IplImage *get_graphed_image()
{
    CvCapture *filecapture = cvCaptureFromFile("trace55.jpg");
    IplImage *obstacles;
    
    cvGrabFrame(filecapture);
    obstacles = cvCloneImage(cvRetrieveFrame(filecapture, 0));
    cvReleaseCapture(&filecapture);
    
    IplImage *grey = same_size_image_8bit(obstacles);
    
    cvCvtColor(obstacles, grey, CV_BGR2GRAY);
    
    CBlobResult blobs = CBlobResult(grey, NULL, 0, true);
    blobs.Filter(blobs, B_INCLUDE, CBlobGetMean(), B_GREATER, 10);
    blobs.Filter(blobs, B_INCLUDE, CBlobGetArea(), B_GREATER, 20);
    blobs.Filter(blobs, B_INCLUDE, CBlobGetArea(), B_LESS, 250000);

    blobs.PrintBlobs("/dev/stdout");
    
    for (int i = 0; i < blobs.GetNumBlobs(); i++)
    {
        CBlob blob = blobs.GetBlob(i);
        CvBox2D box = blob.GetEllipse();
        CvPoint2D32f pt[4];
        
        blob.FillBlob(obstacles, CV_RGB(0,255,0));
        printf("blob %d has %d edges\n", i, blob.edges->total);
        
        cvBoxPoints(box, pt);
        obstacleboxes.push_back(box);
        
        for (int j = 0; j < 4; j++) {
            cvLine(obstacles, cvPointFrom32f(pt[j]), cvPointFrom32f(pt[(j+1)%4]), CV_RGB(0, 0, 255));
        }
        //draw_rect(obstacles, fr, 1);
    }
    
    return obstacles;
}

int main(int argc, char *argv[])
{    
    IplImage *graphed_image;
    
    gStorage    = cvCreateMemStorage(0);
    graphed_image = get_graphed_image();
    
    cvNamedWindow("graphedw", 1);
    cvShowImage("graphedw", graphed_image);
    
    
    
    while (1) {
        cvWaitKey(2000);
    }
    
    return 0;
}