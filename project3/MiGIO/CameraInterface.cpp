#include "CameraInterface.h"
#include "HTTPInterface.h"
#include "RobotInterfaceRovio.h"

#include <stdio.h>
//#include <tchar.h>
#include "Blob.h"
#include "BlobResult.h"
#include "visilibity.hpp"

#include <fstream>
#ifndef _WIN32
#include <unistd.h>
#endif
using namespace std;

static float angle_towards(int x1, int y1, int x2, int y2)
{
    int xv = x2 - x1;
    int yv = y2 - y1;
        
    float res = atan2f(yv, xv);
    
    res = (2*M_PI) - res;
    res = fmodf(res, 2*M_PI);
    return res * (180./M_PI);
}

CvPoint fruitPos, robotPos;
CvPoint targetPos, originalRobotPos;
int robotOrientation;

static int find_objects(bool find_fruit);

// 1 if b is CCW of a
// 0 if b is CW of a
static int compare_angle(float a, float b)
{
    float d = b - a;
    if (d <= 180. && d > 0) {
        return 1;
    }
    return 0;
}

static void robot_turn_from_to(float curAngle, float wantAngle)
{
    float maxA, minA;
    horizontal_class turn;
    float diff, fn;
    int n;
    
    if (compare_angle(curAngle, wantAngle)) {
        turn = TurnLeft;
        maxA = wantAngle;
        minA = curAngle;
    } else {
        turn = TurnRight;
        maxA = curAngle;
        minA = wantAngle;
    }
    
    diff = maxA - minA;

    fn = diff / 15.;
    n = fn + .5;
    
    printf(">> turn %f -> %f (%f degrees dir %d = %f turns = cmd %d)\n", curAngle, wantAngle, diff, (int)turn, fn, n);
    rovio_turn(turn, n);
    
    do {
        horizontal_class cturn;

        // wait to find the robot
        while (find_objects(false) == false)
            ;
        curAngle = robotOrientation;
        if (compare_angle(curAngle, wantAngle)) {
            cturn = TurnLeft;
        } else {
            cturn = TurnRight;
        }
        
        // FIXME make angles_are_closer_than(...) function
        if ((abs(curAngle - wantAngle) <= 10 || abs((360+curAngle) - wantAngle) <= 10))
            break;
        
        if (turn == TurnLeft && wantAngle <= curAngle)
            break;
        if (turn == TurnRight && wantAngle >= curAngle)
            break;
        printf("correction turn\n");
        rovio_turn(turn, 1);
    } while (1);
    
    printf("<< turned, final angle %f wanted %f\n", curAngle, wantAngle);
}

static int distance(int x1, int y1, int x2, int y2)
{
    int xd = x1-x2;
    int yd = y1-y2;
    return xd*xd+yd*yd;
}

// we're already turned as best we can
// just drive forwards until past it
static void robot_drive_to(int wantX, int wantY)
{
    int cdistance, last_distance;
    
    printf(">> drive from %d,%d to %d,%d\n", robotPos.x, robotPos.y, wantX, wantY);
    
    cdistance = distance(robotPos.x, robotPos.y, wantX, wantY);

    do {
        printf("distance = %f, driving forward\n", sqrt(cdistance));
        rovio_drive(5, DirForward);
                
        // wait to find the robot
        // FIXME make this a function
        while (find_objects(false) == false)
            ;
        
        last_distance = cdistance;
        cdistance = distance(robotPos.x, robotPos.y, wantX, wantY);
        printf("distance now = %f\n", sqrt(cdistance));
    } while (cdistance <= last_distance);
    
    printf("<< stopped, distance increased to %f (pos %d, %d)\n", sqrt(cdistance), robotPos.x, robotPos.y);
}

CvMemStorage *gStorage = NULL;
static std::vector<CvBox2D> obstacleboxes;
static VisiLibity::Environment *visibility;

static std::vector<VisiLibity::Point> path_to_goal;

static void drive_to_point(VisiLibity::Point p)
{
    int did_drive;
    int should_redo_turn = 1;
    
    do {
        int next_x = p.x(), next_y = p.y();
        int cur_x = robotPos.x, cur_y = robotPos.y;
        float angle;
        
        did_drive = 0;
        
        do {
            angle = angle_towards(cur_x, cur_y, next_x, next_y);
            float curAngle = robotOrientation;
            
            if (!should_redo_turn)
                break;
            
            if ((abs(curAngle - angle) < 10 || abs((360+curAngle) - angle) < 10))
                break;
                        
            robot_turn_from_to(curAngle, angle);
        } while (0);
        
        do {
            if (distance(cur_x,cur_y,next_x,next_y) <= 100*100)
                break;
            
            did_drive = 1;
            
            robot_drive_to(next_x, next_y);
        } while (0);
        
        // check the angle of the distance
        // if it flipped, we crossed the center point
        // if it didn't, we drove off somewhere - turn around and try again
        float new_angle = angle_towards(robotPos.x, robotPos.y, next_x, next_y);
        float inverse_new_angle = 360. - new_angle;
        
        // FIXME make angles_are_closer_than(...) function
        should_redo_turn = !(abs(inverse_new_angle - angle) < 45 || abs((360+inverse_new_angle) - angle) < 45);
        printf("-- after stopping, old angle = %f, new angle = %f, should be inverse, retry %d, didDrive %d\n", angle, new_angle, should_redo_turn, did_drive);
    } while (should_redo_turn);
}

// drives robot (robotPos) to the goal
static void drive_to_goal()
{
    using namespace VisiLibity;
    for (int i = 1; i < path_to_goal.size(); i++) {
        Point p = path_to_goal[i];
        
        drive_to_point(p);
        return;
    }
}

#pragma mark -- camera control ends here

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

static void draw_point(IplImage *frame, int x, int y, int color)
{
    // drawing
    CvPoint pt1, pt2;
    const CvScalar colors[] = {
        CV_RGB(255, 0, 0), // face red
        CV_RGB(0, 0, 255), // chosen white
    };
    
    pt1.x = x - 5;
    pt2.x = x + 5;
    pt1.y = y - 5;
    pt2.y = y + 5;
    
    cvRectangle(frame, pt1, pt2, colors[color], 3, 8, 0);
}

// returns input image (obstacles highlighted)
// fills out global variable obstacleboxes
static IplImage *visibility_mark_obstacle_boxes(IplImage *obstacles)
{
    IplImage *grey = same_size_image_8bit(obstacles);
    
    cvCvtColor(obstacles, grey, CV_BGR2GRAY);    
    
    CBlobResult blobs = CBlobResult(grey, NULL, 10, true);

    blobs.Filter(blobs, B_INCLUDE, CBlobGetArea(), B_GREATER, 50);
    blobs.Filter(blobs, B_INCLUDE, CBlobGetArea(), B_LESS, 250000);
    
    blobs.PrintBlobs("/dev/stdout");
    
    for (int i = 0; i < blobs.GetNumBlobs(); i++)
    {
        CBlob blob = blobs.GetBlob(i);
        CvBox2D box = blob.GetEllipse();
        CvPoint2D32f pt[4];
                
        {
            CvRect fr;
            fr.x      = blob.MinX();
            fr.width  = blob.MaxX() - fr.x;
            fr.y      = blob.MinY();
            fr.height = blob.MaxY() - fr.y;
            
            if (box.size.width > fr.width)
                box.size.width = fr.width; // some opencv bug
            
            if (box.size.height > fr.height)
                box.size.height = fr.height;
            
            if (fr.width >= obstacles->width && fr.height >= obstacles->height)
                continue;
            
            draw_rect(obstacles, fr, 1);
        }
        
        blob.FillBlob(obstacles, CV_RGB(0,255,0));
        printf("blob %d has %d edges\n", i, blob.edges->total);
        printf("%d - x %f to %f, y %f to %f\n", i, blob.MinX(), blob.MaxX(), blob.MinY(), blob.MaxY());        
        
        cvBoxPoints(box, pt);
        obstacleboxes.push_back(box);
        
        for (int j = 0; j < 4; j++) {
            cvLine(obstacles, cvPointFrom32f(pt[j]), cvPointFrom32f(pt[(j+1)%4]), CV_RGB(0, 0, 255));
        }
    }
    
    return obstacles;
}

static VisiLibity::Point move_point_away(VisiLibity::Point p)
{
    // find the obstacle whose corner this point is on
    // then move the point away from its center
    float cx=0, cy=0;
    
    for (int i = 0; i < obstacleboxes.size(); i++) {
        CvBox2D box = obstacleboxes[i];
        CvPoint2D32f pt[4];
        
        cvBoxPoints(box, pt);
        
        for (int j = 0; j < 4; j++) {
            float px = pt[j].x, py = pt[j].y;
            
            if (abs(px - p.x()) < 2. && abs(py - p.y()) < 2.) {
                cx = box.center.x;
                cy = box.center.y;
                goto done;
            }
        }
    }
    
    return p;
    
done:
    float cxv = p.x() - cx, cyv = p.y() - cy;
    float r = sqrtf(cxv * cxv + cyv * cyv);
    cxv /= r;
    cyv /= r;
    
    cxv *= 35;
    cyv *= 35;
    
    return VisiLibity::Point(p.x() + cxv, p.y() + cyv);
}

// input - visibility_mark_obstacle_boxes
// output - visibility graph drawn over image
// fills out global variable visibility
static IplImage *visibility_make_visibility_graph(IplImage *obstacle_image)
{
    using namespace VisiLibity;
    
    {
        Point rp[4] = {
            Point(0, 0),
            Point(obstacle_image->width, 0),
            Point(obstacle_image->width, obstacle_image->height),
            Point(0, obstacle_image->height)};
        std::vector<Point> rpv(rp, rp+4);
        Polygon boundary(rpv);
        
        visibility = new Environment(boundary);
    }
    
    for (int i = 0; i < obstacleboxes.size(); i++) {
        CvBox2D box = obstacleboxes[i];
        CvPoint2D32f pt[4];
        Point pp[4];
        
        cvBoxPoints(box, pt);
        
        for (int j = 0; j < 4; j++) {
            pp[j] = Point(pt[j].x, pt[j].y);
        }
        
        std::vector<Point> pv(pp, pp+4);
        Polygon hole(pv);
        visibility->add_hole(hole);
    }
    
    visibility->enforce_standard_form();
    
    //printf("vis valid %d area %f dia %f\n", visibility->is_valid(), visibility->area(), visibility->diameter());
    
    //printf("drawing visibility graph\n");
    
    IplImage *visibility_image = cvCloneImage(obstacle_image);
    
    int nlines = 0, nvertices = 0;
    
    {
        Visibility_Graph graph(*visibility);
        nvertices = graph.n();
        
        for (int i = 0; i < nvertices; i++) {
            Point &p1 = (*visibility)(i);
            for (int j = 0; j < nvertices; j++) {
                Point &p2 = (*visibility)(j);
                if (!graph(i, j) || (j > i && graph(j, i))) continue;
                
                nlines++;
                
                CvPoint cp1, cp2;
                cp1.x = p1.x();cp1.y = p1.y();
                cp2.x = p2.x();cp2.y = p2.y();
                
                cvLine(visibility_image, cp1, cp2, CV_RGB(255, 0, 255));
            }
        }
        
    }
    
    printf("%d vertices, %d edges in graph\n", nvertices, nlines);
    
    return visibility_image;
}

// input - visibility_make_visibility_graph
// global variables robotPos.x, robotPos.y, targetPos.x, targetPos.y
// output - image with path drawn on it
// global variable path_to_goal
static IplImage *visibility_find_robot_path(IplImage *visibility_graph_image)
{
    using namespace VisiLibity;

    //printf("drawing shortest path\n");
    Point robotp(robotPos.x, robotPos.y);
    Point goalp(targetPos.x, targetPos.y);
    Polyline path = visibility->shortest_path(robotp, goalp, 5);
    
    path_to_goal.clear();
    
    for (int i = 0; i < path.size(); i++) {
        Point p = move_point_away(path[i]);
        path_to_goal.push_back(p);
    }
    
    for (int i = 0; i < (path.size()-1); i++) {
        Point p  = path_to_goal[i];
        Point p1 = path_to_goal[i+1];
        CvPoint cp1, cp2;
        cp1.x = p.x(); cp1.y = p.y();
        cp2.x = p1.x();cp2.y = p1.y();
        
        printf("p #%d: x %d y %d -> x %d y %d\n", i, cp1.x, cp1.y, cp2.x, cp2.y);
        
        cvLine(visibility_graph_image, cp1, cp2, CV_RGB(255, 255, 255), 4, 8, 0);
    }
    
    return visibility_graph_image;
}

IplImage* img = NULL;
int numCameras=0;
IplImage* background = NULL;
IplImage* ObstacleBackground = NULL;
IplImage* visibility_image = NULL;
char cameraName[20][1000];
char cameraURL[20][50000];

//std::vector<CvPoint> path;
//std::vector<CvPoint> robotPos;
int imageCount = 0;

void setBackground(){
	int key;
	IplImage* img = NULL;
    
	while(1){
        key = cvWaitKey(30);
        http_fetch(cameraURL[0],"Data/Camera0.jpg");
        img=cvLoadImage("Data/Camera0.jpg");
        cvShowImage("Display", img);
        if(key == '1')
            break;
        cvReleaseImage(&img);
	}
    background = cvCloneImage(img);
	cvSmooth(img, background, CV_BILATERAL,5,5,50,50);
	cvReleaseImage(&img);
}

void setObstacleBackground(){
    int key, backr, backg, backb;
    IplImage* img = NULL;

	while(1){
        key = cvWaitKey(30);
        http_fetch(cameraURL[0],"Data/Camera0.jpg");
        img=cvLoadImage("Data/Camera0.jpg");
        cvShowImage("Display" , img);
        if(key == '2')
            break;
		cvReleaseImage(&img);
	}
    
    ObstacleBackground = cvCloneImage(img);
	cvSmooth(img, ObstacleBackground, CV_BILATERAL,5,5,50,50);
    cvReleaseImage(&img);
    img = cvCloneImage(ObstacleBackground);

	for(int i = 0; i < img->height; i++) {
		for(int j = 0; j < img->width; j++) {
			int r = ((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 2];
            int g = ((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 1];
            int b = ((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 0];
			backr = ((uchar *)(background->imageData + i*img->widthStep))[j*img->nChannels + 2];
			backg = ((uchar *)(background->imageData + i*img->widthStep))[j*img->nChannels + 1];
			backb = ((uchar *)(background->imageData + i*img->widthStep))[j*img->nChannels + 0];
            
            int diff = (r-backr)*(r-backr) + (g-backg)*(g-backg)+ (b-backb)*(b-backb);
            
            diff = sqrt(diff);
            if (diff > 255) diff = 255;
            if (diff < 0) diff = 0;
            
            if (diff > 20) diff = 255;
            
            ((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 2] = diff;
            ((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 1] = 0;
            ((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 0] = 0;
		}
	}
        
    cvErode(img, img);
    cvDilate(img, img);
    
    img = visibility_mark_obstacle_boxes(img);
    img = visibility_make_visibility_graph(img);
    
    visibility_image = img;
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
    int black = (r+g+b) < 25;
    if (black) return 0;
	int diff = (FilteredColor[0] - r)*(FilteredColor[0]-r)+
                (FilteredColor[1] - g)*(FilteredColor[1] - g)+
                (FilteredColor[2] - b)*(FilteredColor[2] - b);

    diff = sqrt(diff);
    if (diff > 255) diff = 255;
    if (diff < 0) diff = 0;
    diff = 255 - diff;
    
    if (diff <= 180) diff = 0;
    
    return diff;
}

int RGB_filter2(int r, int g, int b, int threshold){
	int FilteredColor[3] = {40, 60, 160}; //the RGB values for the blue felt.

    int black = (r+g+b) < 25;
    if (black) return 0;
    
	int diff = (FilteredColor[0] - r)*(FilteredColor[0]-r)+
                (FilteredColor[1] - g)*(FilteredColor[1] - g)+
                (FilteredColor[2] - b)*(FilteredColor[2] - b);

    diff = sqrt(diff);
    if (diff > 255) diff = 255;
    if (diff < 0) diff = 0;
    diff = 255 - diff;
    
    if (diff <= 180) diff = 0;

    return diff;
}

int RGB_filter3(int r, int g, int b, int threshold){
	//int FilteredColor[3] = {240, 70, 120}; //the RGB values for teh apple.
	int FilteredColor[3] = {253, 249, 149}; // the RGB values for teh lemon
    
    int black = (r+g+b) < 25;
    if (black) return 0;
    
	int diff = (FilteredColor[0] - r)*(FilteredColor[0]-r)+
                (FilteredColor[1] - g)*(FilteredColor[1] - g)+
                (FilteredColor[2] - b)*(FilteredColor[2] - b);

    diff = sqrt(diff);
    if (diff > 255) diff = 255;
    if (diff < 0) diff = 0;
    diff = 255 - diff;
    return diff;
}

// finds robot and fruit
static int find_objects(bool find_fruit)
{
	IplImage* input;
    IplImage* img, *imgCopy;
    IplImage* i1;
	IplImage* rob;
	IplImage* fruit;

    CvPoint Left, Right;
    
	http_fetch(cameraURL[0],"Data/Camera0.jpg");
    //	cvReleaseImage(&images[0]);
	input=cvLoadImage("Data/Camera0.jpg");
	//cvShowImage(cameraName[0], input);
    
    cvDestroyWindow("Display");
    
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
    imgCopy = cvCloneImage(img);
    cvSmooth(imgCopy, img, CV_BILATERAL,5,5,50,50);
    cvReleaseImage(&imgCopy);
            
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
    
	//cvShowImage("Background subtracted", img);
	for(int i = 0; i < img->height; i++) {
		for(int j = 0; j < img->width; j++) {
			int r = ((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 2];
            int g = ((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 1];
            int b = ((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 0];

            ((uchar *)(i1->imageData + i*i1->widthStep))[j] = RGB_filter1(r, g, b, 6000);
			((uchar *)(rob->imageData + i*rob->widthStep))[j] = RGB_filter2(r, g, b, 6000);
            ((uchar *)(fruit->imageData + i*fruit->widthStep))[j] = RGB_filter3(r,g,b, 6000);
        }
    }
    
    int found_robot=0, found_fruit=0;
    
    cvErode(i1, i1, NULL, 2);
    cvErode(rob, rob, NULL, 2);
    cvErode(fruit, fruit);
    
    /*
    cvShowImage("robotLeft", rob);
    cvShowImage("robotRight", i1);
    
     cvShowImage("fruit", fruit);
    */
    
    CBlobResult robBlobs = CBlobResult(rob, NULL, 0, true);
    CBlobResult blobs = CBlobResult(i1, NULL, 0, true);
    CBlobResult fruitBlob = CBlobResult(fruit, NULL, 0, true);
    
    fruitBlob.Filter(fruitBlob, B_INCLUDE, CBlobGetMean(), B_GREATER, 120);
    fruitBlob.Filter(fruitBlob, B_INCLUDE, CBlobGetArea(), B_GREATER, 20);
    fruitBlob.Filter(fruitBlob, B_INCLUDE, CBlobGetArea(), B_LESS, 6000);

    blobs.Filter(blobs, B_INCLUDE, CBlobGetMean(), B_GREATER, 130);
    blobs.Filter(blobs, B_INCLUDE, CBlobGetArea(), B_GREATER, 50);
    blobs.Filter(blobs, B_INCLUDE, CBlobGetArea(), B_LESS, 6000);
    
    robBlobs.Filter(robBlobs, B_INCLUDE, CBlobGetMean(), B_GREATER, 130);
    robBlobs.Filter(robBlobs, B_INCLUDE, CBlobGetArea(), B_GREATER, 20);
    robBlobs.Filter(robBlobs, B_INCLUDE, CBlobGetArea(), B_LESS, 6000);

    // find right side of robot
    for (int i = 0; i < blobs.GetNumBlobs(); i++ )
    {
        CBlob blobArea = blobs.GetBlob(i);
        CvBox2D BlobEllipse = blobArea.GetEllipse();
        CvPoint centrum = cvPoint(BlobEllipse.center.x, BlobEllipse.center.y);

        Right = centrum;
        found_robot = 1;
        if(blobArea.Area() > 10)
        { blobArea.FillBlob(input, cvScalar(255, 0, 0));
            cvCircle(input, centrum, 20, CV_RGB(250, 250, 0),2, 1);}
    }
    
    // find fruit
    for (int i = 0; i < fruitBlob.GetNumBlobs(); i++ )
    {
        CBlob fruitBlobArea = fruitBlob.GetBlob(i);
        CvBox2D BlobEllipse = fruitBlobArea.GetEllipse();
        CvPoint centrum = cvPoint(BlobEllipse.center.x, BlobEllipse.center.y);

        fruitPos = centrum;
        found_fruit = 1;
        if(fruitBlobArea.Area() > 10)
        { fruitBlobArea.FillBlob(input, cvScalar(255, 0, 250));
            cvCircle(input, centrum, 20, CV_RGB(250, 0, 250),2, 1);}
    }
        
    //robotPos.clear();
    //find other side of robot
    for (int i = 0; i < robBlobs.GetNumBlobs(); i++ )
    {
        //printf("blobbled");
        CBlob robBlobArea = robBlobs.GetBlob(i);
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
        if (found_robot==1)
            found_robot++;
        
        if(robBlobArea.Area() > 4)
        { robBlobArea.FillBlob(input, cvScalar(0, 255, 0));
            cvCircle(input, centrum, 20, CV_RGB(250, 250, 0),2, 1);}
    }
    
    if(found_robot == 2){
        cvCircle(input, Right, 20, CV_RGB(250, 250, 250),2,1);
        cvCircle(input, Left, 20, CV_RGB(127, 127, 127), 2, 1);
        cvCircle(input, fruitPos, 20, CV_RGB(0, 0, 0), 2, 1);
        double temp = sqrt(distance(Left.x,Left.y,Right.x,Right.y));
        temp = acos((Right.x-Left.x)/temp);
        temp = (temp/M_PI)*180;

        if(((Right.y - Left.y) < 0) && ((Right.x-Left.x) > 0)){
            temp = 360 -temp;
        }
        else if(((Right.y - Left.y < 0) && ((Right.x - Left.x) < 0))){
            float rho = 180 - temp;
            temp = temp + 2*rho;
        }
        temp = temp + 90;
        temp = fmod(temp,360);
        if(temp > 180){
            temp = (180 + 360 - temp);
        }
        if(temp < 180){
            temp = 180 - temp;
        }
        robotOrientation = temp;
        
        robotPos.x = (int)(Right.x + Left.x)/2;
        robotPos.y = (int)(Right.y + Left.y)/2;
        /*
        double fruitAngle = sqrt(distance(fruitPos.x, fruitPos.y, robotPos.x, robotPos.y));
        fruitAngle = acos((fruitPos.x - robotPos.x)/fruitAngle) * (180./M_PI);

        if(((fruitPos.y - robotPos.y) > 0) && ((fruitPos.x-robotPos.x) > 0)){
            fruitAngle = 360 -fruitAngle;
        }
        else if(((fruitPos.y - robotPos.y > 0) && ((fruitPos.x - robotPos.x) < 0))){
            float rho = 180 - fruitAngle;
            fruitAngle = fruitAngle + 2*rho;
        }
        
        int difference = (fruitAngle - robotOrientation);
        if(difference > 180){
            difference = difference - 360;
        }
        if(difference < -180){
            difference = difference + 360;
        }
        */
        /*
        CvPoint relativeFruitPos;
        relativeFruitPos.x = (int)cos((double)((difference/360)*M_PI)) * 8;
        relativeFruitPos.y = (int)sin((double)((difference/360)*M_PI)) * 8;
        */
        
        //printf("The orientation is: %d \n", difference);
    }
 
    blobs.ClearBlobs();
    cvReleaseImage(&img);

    cvReleaseImage(&fruit);
    cvReleaseImage(&rob);
    robBlobs.ClearBlobs();
    cvReleaseImage(&i1);
    
    CvScalar cyan = CV_RGB(0, 250,250);
    //CvScalar lightGreen = CV_RGB(127,255,0);
    //CvScalar purple = CV_RGB(138,43,226);
    
    // here is where we are going to draw in our squares.
    /*
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
        
        //int shapeSize = 100;
        for(int i = 0; i < path.size(); i ++){
            cvCircle(input, path.at(i),1, cyan, 2);
        }
    }
    */
 
   // cvNamedWindow("Output Image - Blob Demo", 1);
    {
        static int localCount = 0;
        if(localCount % 5 == 0 && 0){
            localCount = 0;
            string imageName;
            std::stringstream out;
            out << "trace/trace" << imageCount <<".jpg";
            imageName = out.str();
            cvSaveImage(imageName.c_str(), input);
            imageCount= imageCount + 1;
        }
        localCount++;
    }
    
    cvShowImage("Output Image - Blob Demo", input);
    cvWaitKey(3);

    cvReleaseImage(&input);
    
    int sdistance = distance(fruitPos.x, fruitPos.y, robotPos.x, robotPos.y);
    
    printf("found robot %d (%d,%d), angle %d, fruit %d (%d,%d), dist %f\n", found_robot, robotPos.x, robotPos.y, robotOrientation, found_fruit, fruitPos.x, fruitPos.y, sqrt(sdistance));
    
    return found_robot==2 && (!find_fruit || (found_fruit && (sdistance > 50*50)));
}

static void idleAwaitingObjects()
{
 	int key;
	IplImage* img = NULL;
    
	while(1){
        key = cvWaitKey(30);
        http_fetch(cameraURL[0],"Data/Camera0.jpg");
        img=cvLoadImage("Data/Camera0.jpg");
        cvShowImage("Display", img);
        cvReleaseImage(&img);
        if(key == '3')
            break;
	} 
}

void processCamera()
{
    static int first = 1, placed = 0, found=0;

    if(first){
        first = 0;
        setBackground();
        setObstacleBackground();
	}
    
    cvShowImage("visibility graph", visibility_image);

    if (!placed) {
        idleAwaitingObjects();
        placed = 1;
    }
    
    if (!found && find_objects(true)) {
        found = 1;
        originalRobotPos = robotPos;
        targetPos = fruitPos;
        visibility_image = visibility_find_robot_path(visibility_image);
        cvShowImage("visibility graph", visibility_image);
        
        rovio_camera_height(middle);
        
        printf("--\ndrive to fruit\n\n");
        drive_to_goal();
        
        rovio_camera_height(low);
        
        printf("--\ndrive back\n\n");
        targetPos = originalRobotPos;
        visibility_find_robot_path(NULL);
        drive_to_goal();
        
    }
}

void initCameras(){
	// Initialize all the cameras from infile
	char cname[100];
	fstream infile("Data/config-cam.txt",ios::in);
	while(!infile.eof()){
		infile >> cameraURL[numCameras];
		sprintf(cname, "Data/Camera%d.jpg", numCameras);
		sprintf(cameraName[numCameras], "Data/Camera%d.jpg", numCameras);
        
	/*	cvNamedWindow(cname);
		http_fetch(cameraURL[numCameras],cname);
		images[numCameras] = cvLoadImage(cname);
		cvShowImage(cameraName[numCameras], images[numCameras]);
        */
		numCameras++;
	}
	infile.close();
}

IplImage* images[20];

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
			if (!images[i]) continue;
			cvShowImage(cameraName[i], images[i]);
		}
#ifdef _WIN32
		Sleep(100);
#else
		usleep(100);
#endif
	}
}

